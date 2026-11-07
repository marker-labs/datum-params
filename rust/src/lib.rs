//! Shared evaluation core for Datum parameters, used by compiled artifact generation
//! and by in-process lazy evaluation so both modes run identical semantics.
pub mod lazy_capi;
use serde::Deserialize;
use anyhow::{bail, Context, Result};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, fs, io::{Cursor, Read}, path::{Component, Path, PathBuf}};

#[derive(Deserialize)]
pub struct Mapping { pub path: String, pub pointer: String, pub source_units: String, pub unit: String, pub min: Option<f64>, pub max: Option<f64> }
#[derive(Deserialize)]
pub struct Derived { pub operation: String, pub inputs: Vec<String>, pub unit: String, pub min: Option<f64>, pub max: Option<f64> }
#[derive(Deserialize)]
pub struct Constant { pub value: f64, pub unit: String, pub min: Option<f64>, pub max: Option<f64> }
#[derive(Deserialize)]
pub struct Worktree { #[serde(default)] pub constants: BTreeMap<String, Constant>, #[serde(default)] pub parameters: BTreeMap<String, Mapping>, #[serde(default)] pub derived: BTreeMap<String, Derived>, #[serde(default)] pub groups: BTreeMap<String, Vec<String>>, #[serde(default)] pub sources: Vec<String> }
#[derive(Deserialize)]
pub struct Spec { pub schema: String, pub units: String, #[serde(default)] pub worktrees: BTreeMap<String, Worktree>, #[serde(default)] pub sources: Vec<String>, #[serde(flatten)] pub metadata: BTreeMap<String, Value> }
pub fn ident(s:&str)->bool { !s.is_empty() && !s.starts_with(|c:char| c.is_ascii_digit()) && s.chars().all(|c|c.is_ascii_lowercase()||c.is_ascii_digit()||c=='_') }
pub fn safe_relative(root:&Path,name:&str)->Result<PathBuf>{let path=Path::new(name); if path.components().any(|p|!matches!(p,Component::Normal(_))){bail!("invalid source path {name}")} let base=root.canonicalize()?; let path=root.join(path).canonicalize()?;if !path.starts_with(base){bail!("source escapes project: {name}")} Ok(path)}
pub fn check_bounds(name:&str,v:f64,min:Option<f64>,max:Option<f64>)->Result<()> {if !v.is_finite()||min.is_some_and(|x|!x.is_finite())||max.is_some_and(|x|!x.is_finite())||min.zip(max).is_some_and(|(a,b)|a>b){bail!("invalid value or bounds for {name}")} if min.is_some_and(|x|v<x)||max.is_some_and(|x|v>x){bail!("{name}={v} is outside allowed bounds [{}, {}]",min.map_or_else(||"-inf".into(),|x|x.to_string()),max.map_or_else(||"+inf".into(),|x|x.to_string()))} Ok(())}
fn parse_quantity(text:&str, default_unit:&str)->Result<f64>{
    let text=text.trim();
    let (number,unit)=if let Ok(value)=text.parse::<f64>(){(value,default_unit)}else{
        let parts:Vec<&str>=text.split_whitespace().collect();
        if parts.len()==2{(parts[0].parse::<f64>().with_context(||format!("unsupported SolidCraft expression '{text}'"))?,parts[1])}
        else{
            let mut parsed=None;
            for unit in ["mm","cm","in","ft","m"]{
                if let Some(number)=text.strip_suffix(unit){
                    if let Ok(value)=number.trim().parse::<f64>(){parsed=Some((value,unit));break}
                }
            }
            parsed.with_context(||format!("unsupported SolidCraft expression '{text}'; use a numeric literal with an optional unit"))?
        }
    };
    Ok(number*unit_scale(unit,default_unit).with_context(||format!("unsupported SolidCraft unit '{unit}'"))?)
}

pub fn solidcraft_number(data:&Value,item:&Value,default_unit:&str)->Result<f64>{
    if let Some(value)=item.as_f64(){return Ok(value)}
    let expression=item.as_str().context("CAD dimension is not numeric")?;
    if let Ok(value)=parse_quantity(expression,default_unit){return Ok(value)}
    let parameter=data.get("params").and_then(Value::as_array).and_then(|params|
        params.iter().find(|p|p.get("name").and_then(Value::as_str)==Some(expression))
    ).with_context(||format!("'{expression}' is not a numeric literal or a named SolidCraft parameter"))?;
    let value=parameter.get("expr").and_then(Value::as_str).with_context(||format!("SolidCraft parameter '{expression}' has no expression"))?;
    let unit=parameter.get("unit").and_then(Value::as_str).unwrap_or(default_unit);
    parse_quantity(value,unit).and_then(|value|unit_scale(unit,default_unit).map(|scale|value*scale))
        .with_context(||format!("SolidCraft parameter '{expression}' must resolve to a numeric unit literal"))
}

pub fn cad_value(root:&Path,m:&Mapping,hashes:&mut BTreeMap<String,String>)->Result<f64>{
    let path=safe_relative(root,&m.path)?;
    let bytes=fs::read(path)?;
    let mut archive=zip::ZipArchive::new(Cursor::new(&bytes)).with_context(||format!("{} is not a SolidCraft archive",m.path))?;
    let mut doc=archive.by_name("document.json")?;
    if doc.size()>32*1024*1024{bail!("SolidCraft document too large")}
    let mut text=String::new();doc.read_to_string(&mut text)?;
    let data:Value=serde_json::from_str(&text)?;
    if data.get("units").and_then(Value::as_str)!=Some(m.source_units.as_str()){bail!("units mismatch: {}",m.path)}
    let item=data.pointer(&m.pointer).with_context(||format!("{} has no feature/dimension at {}",m.path,m.pointer))?;
    let n=solidcraft_number(&data,item,&m.source_units).with_context(||format!("read CAD dimension {}",m.pointer))?;
    let scale=unit_scale(&m.source_units,&m.unit)?;
    let value=n*scale;
    check_bounds("CAD dimension",value,m.min.map(|v|v*scale),m.max.map(|v|v*scale))?;
    let digest=format!("{:x}",Sha256::digest(&bytes));
    if hashes.get(&m.path).is_some_and(|previous| previous != &digest){bail!("source changed during evaluation: {}",m.path)}
    hashes.insert(m.path.clone(),digest);
    Ok(value)
}

/// One lazy/compiled evaluation: spec + saved CAD sources -> checked parameter values.
pub struct Evaluation {
    pub root: PathBuf,
    pub spec_path: PathBuf,
    pub worktree: String,
    pub units: String,
    pub parameters: BTreeMap<String, f64>,
    pub groups: BTreeMap<String, Vec<String>>,
    pub bounds: BTreeMap<String, Value>,
    pub metadata: Value,
    pub sources_sha256: BTreeMap<String, String>,
    pub spec_sha256: String,
}

pub fn unit_scale(source:&str,target:&str)->Result<f64>{
    let meters=|unit:&str|->Option<f64>{match unit{"mm"=>Some(0.001),"cm"=>Some(0.01),"m"=>Some(1.0),"in"=>Some(0.0254),"ft"=>Some(0.3048),_=>None}};
    Ok(meters(source).context("unsupported source unit")?/meters(target).context("unsupported target unit")?)
}

pub fn evaluate(root:&Path,spec_path:&Path,worktree_name:&str)->Result<Evaluation>{
    let spec_bytes=fs::read(spec_path)?;let spec:Spec=serde_json::from_slice(&spec_bytes).context("read parameter specification")?;if spec.schema!="datum.robot.parameter-spec/2"||spec.units!="m"{bail!("expected datum.robot.parameter-spec/2 with canonical units m")}if !ident(worktree_name){bail!("invalid worktree name")};let tree=spec.worktrees.get(worktree_name).with_context(||format!("worktree '{worktree_name}' is not defined in the spec"))?;let mut values=BTreeMap::<String,f64>::new();let mut hashes=BTreeMap::new();let mut bounds=BTreeMap::<String,Value>::new();let groups=tree.groups.clone();
    for(name,c)in &tree.constants{if !ident(name)||c.unit!="m"{bail!("invalid constant {name}")};check_bounds(name,c.value,c.min,c.max)?;if c.value<=0.0 && !c.min.is_some_and(|x|x<=0.0){bail!("invalid non-positive Datum parameter {name}")};values.insert(name.clone(),c.value);bounds.insert(name.clone(),json!({"min":c.min,"max":c.max,"allow_nonpositive":c.min.is_some_and(|x|x<=0.0)}));}
    for(name,m)in &tree.parameters{if !ident(name){bail!("invalid parameter name {name}")}let v=cad_value(root,m,&mut hashes)?;if v<=0.0{bail!("CAD dimension {name} must be positive")};check_bounds(name,v,m.min.map(|x|x*unit_scale(&m.source_units,&m.unit).unwrap()),m.max.map(|x|x*unit_scale(&m.source_units,&m.unit).unwrap()))?;bounds.insert(name.clone(),json!({"min":m.min.map(|x|x*unit_scale(&m.source_units,&m.unit).unwrap()),"max":m.max.map(|x|x*unit_scale(&m.source_units,&m.unit).unwrap())}));values.insert(name.clone(),v);}
    for(name,d)in &tree.derived{if !ident(name){bail!("invalid parameter name {name}")}if d.inputs.is_empty()||d.inputs.iter().any(|i|!values.contains_key(i)){bail!("{name} references a missing input")};let args:Vec<f64>=d.inputs.iter().map(|i|values[i]).collect();let v=match d.operation.as_str(){"sum"=>args.iter().sum(),"product"=>args.iter().product(),"min"=>args.iter().copied().fold(f64::INFINITY,f64::min),"max"=>args.iter().copied().fold(f64::NEG_INFINITY,f64::max),"difference" if args.len()==2=>args[0]-args[1],"ratio" if args.len()==2=>args[0]/args[1],_=>bail!("unsupported operation or arity for {name}")};if d.unit!="m"{bail!("derived parameter {name} must use canonical unit m")};check_bounds(name,v,d.min,d.max)?;if v<=0.0 && !d.min.is_some_and(|x|x<=0.0){bail!("invalid non-positive Datum parameter {name}")};bounds.insert(name.clone(),json!({"min":d.min,"max":d.max,"allow_nonpositive":d.min.is_some_and(|x|x<=0.0)}));values.insert(name.clone(),v);}
    for source in tree.sources.iter().chain(spec.sources.iter()){let data=fs::read(safe_relative(root,source)?)?;let digest=format!("{:x}",Sha256::digest(&data));if hashes.get(source).is_some_and(|previous|previous!=&digest){bail!("source changed during evaluation: {source}")}hashes.insert(source.clone(),digest);}
    for (source, previous) in &hashes {let data=fs::read(safe_relative(root,source)?)?;if format!("{:x}",Sha256::digest(&data))!=*previous{bail!("source changed during evaluation: {source}")}}
    for(group,names)in &groups{if !ident(group)||names.is_empty()||names.iter().any(|n|!values.contains_key(n)){bail!("invalid group {group}: every referenced parameter must exist")}}
    Ok(Evaluation{
        root:root.to_path_buf(),
        spec_path:spec_path.to_path_buf(),
        worktree:worktree_name.to_string(),
        units:spec.units.clone(),
        parameters:values,
        groups:groups.clone(),
        bounds,
        metadata:serde_json::to_value(&spec.metadata)?,
        sources_sha256:hashes,
        spec_sha256:format!("{:x}",Sha256::digest(&spec_bytes)),
    })
}

impl Evaluation {
    /// The canonical manifest document; identical to the compiled-mode output.
    pub fn manifest(&self)->Value{json!({"schema":"datum.robot.parameters/2","units":self.units,"worktree":self.worktree,"parameters":self.parameters,"groups":self.groups,"bounds":self.bounds,"metadata":self.metadata,"sources_sha256":self.sources_sha256,"spec_sha256":self.spec_sha256})}
    pub fn header(&self)->String{
        let mut header=format!("// Generated by datum-params for worktree {}. Regenerate after CAD changes.\n#pragma once\nnamespace datum::robot::{} {{\n",self.worktree,self.worktree);
        for(name,v)in &self.parameters{header.push_str(&format!("inline constexpr double {name} = {v:.17};\n"));}
        header.push_str("}\n");header
    }
    /// Machine-readable evidence record for a lazy evaluation. Records the mode, the
    /// exact values consumed and the source hashes they were extracted from.
    pub fn evidence(&self)->Value{
        let mut evidence=self.manifest();
        evidence["evidence_schema"]=json!("datum.robot.lazy-evidence/1");
        evidence["mode"]=json!("lazy");
        evidence["spec_path"]=json!(self.spec_path.to_string_lossy());
        evidence
    }
    pub fn write_artifacts(&self,manifest_path:&Path,header_path:&Path)->Result<()>{
        if let Some(p)=manifest_path.parent(){fs::create_dir_all(p)?}
        if let Some(p)=header_path.parent(){fs::create_dir_all(p)?}
        fs::write(manifest_path,format!("{}\n",serde_json::to_string_pretty(&self.manifest())?))?;
        fs::write(header_path,self.header())?;
        Ok(())
    }
}
