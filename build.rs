use std::path::PathBuf;
use std::str::FromStr;
use std::{env, fs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=src/lis2mdl.yaml");

    let manifest = fs::read_to_string("src/lis2mdl.yaml")?;
    let generated = device_driver_generation::transform_yaml(&manifest, "Lis2mdlDevice");

    let tokens = proc_macro2::TokenStream::from_str(&generated)?;
    let syntax_tree = syn::parse2::<syn::File>(tokens)?;
    let formatted = prettyplease::unparse(&syntax_tree);

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out_dir.join("lis2mdl_device.rs"), formatted)?;
    Ok(())
}
