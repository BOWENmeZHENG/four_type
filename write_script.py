def write_pymodel(r_out=0.3, r_in=0.2, width=0.1, spoke_width=0.04, num_spokes=3, init_angle=0,
                  load=10000, meshsize=0.03, r_depth=0.02, r_pressure=0.1,
                  results_location='C:/Users/bowen/Desktop/abaqus_python/four_type/',
                  part_name='wheel', material_name='AL', section_name='wheel_section',
                  assembly_name='wheel-assembly', step_name='static_load', load_name='compression',
                  bc_name='fixed', job_name='wheel_compression'):

    filename = f'ro_{r_out:.3f}_ri_{r_in:.3f}_w_{width:.3f}_sw_{spoke_width:.3f}_n_{num_spokes}_m_{material_name}_l_{load:.1f}_rot_{init_angle:.1f}'

    with open(results_location + filename + '.py', 'w') as f:
        f.write("import abaqus_utils as ut\n")
        # Derived values
        f.write(f"s_pt_whole, s_pt_lateral, s_pt_extr, s_pt_out_edge, spoke_start, s_pts_spoke = ut.derived_values({r_in}, {r_out}, {width}, {spoke_width})\n")
        # Define wheel geometry
        f.write("mymodel = mdb.ModelFromInputFile(inputFileName='C:/Users/bowen/Desktop/abaqus_python/four_type/abaqus_al_mat_share.inp', name='Model-1')\n")
        f.write(f"mypart = ut.init_part(mymodel, {r_out}, {r_in}, {width}, '{part_name}')\n")
        f.write(f"ut.spoke(mymodel, mypart, {width}, {num_spokes}, {spoke_width}, {init_angle}, spoke_start, s_pts_spoke, s_pt_extr, s_pt_out_edge)\n")
        # Set for exterior nodes
        f.write("mypart.Set(faces=mypart.faces.getByBoundingSphere(center=(0, 0, 0), radius=10.0), name='all_faces')\n")
        # Material & Section
        f.write(f"ut.mat_sect(mymodel, mypart, '{material_name}', '{section_name}', s_pt_whole)\n")
        # Assembly
        f.write(f"myassembly = ut.make_assembly(mymodel, mypart, '{assembly_name}')\n")
        # Step
        f.write(f"mymodel.StaticStep(name='{step_name}', previous='Initial')\n")
        # Mesh
        f.write(f"ut.make_mesh(mypart, {meshsize}, s_pt_whole, {r_out}, {width})\n")
        # Loading & BC
        f.write(f"ut.load_bc(mymodel, mypart, myassembly, '{step_name}', '{load_name}', '{bc_name}', {r_out}, {width}, {r_depth}, {r_pressure}, {load}, s_pt_lateral)\n")
        # Job
        f.write(f"ut.job('{job_name}')\n")
        # Access results
        f.write(f"nodalS11 = ut.post_process('{job_name}')\n")
        # csv files for ML
        f.write(f"ut.output_csv(mypart, '{results_location}', nodalS11, '{filename}')\n")
    return filename





