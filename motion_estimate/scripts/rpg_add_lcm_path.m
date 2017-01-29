function rpg_add_lcm_path()
base_dir = getenv('DRC_BASE');
jar_dir = [base_dir '/software/build/share/java/'];

javaaddpath([jar_dir 'lcmtypes_bot2-core.jar']);
javaaddpath([jar_dir 'lcmtypes_maps.jar']);
javaaddpath([jar_dir 'lcmtypes_drc_lcmtypes.jar']);
javaaddpath([jar_dir 'lcmtypes_visualization.jar']);
javaaddpath /usr/local/share/java/lcm.jar;