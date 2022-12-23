import abaqus_utils as ut

# Constants
r_out = 0.3
r_in = 0.2
width = 0.1
spoke_width = 0.04
num_spokes = 3
meshsize = 0.03
r_depth = 0.02
r_pressure = 0.1
E = 1e8
mu = 0.3
load = 10000
results_location = 'C:/Users/bowen/Desktop/abaqus_python/parameterised/'

# Names
part_name = 'wheel'
material_name = 'wheel_material'
section_name = 'wheel_section'
assembly_name = 'wheel-assembly'
step_name = 'static_load'
load_name = 'compression'
bc_name = 'fixed'
job_name = 'wheel_compression'

# Derived values
s_pt_whole, s_pt_lateral, s_pt_extr, s_pt_out_edge, spoke_start, s_pts_spoke = ut.derived_values(r_in, r_out, width,
                                                                                                 spoke_width)

# Define wheel geometry
mymodel = mdb.models['Model-1']
mypart = ut.init_part(mymodel, r_out, r_in, width, part_name)
ut.spoke(mymodel, mypart, width, num_spokes, spoke_width, spoke_start, s_pts_spoke, s_pt_extr, s_pt_out_edge)

# set for exterior nodes
mypart.Set(faces=mypart.faces.getByBoundingSphere(center=(0, 0, 0), radius=10.0), name='all_faces')

# Material & Section
ut.mat_sect(mymodel, mypart, material_name, E, mu, section_name, s_pt_whole)

# Assembly
myassembly = ut.make_assembly(mymodel, mypart, assembly_name)

# Step
mymodel.StaticStep(name=step_name, previous='Initial')

# Mesh
ut.make_mesh(mypart, meshsize, s_pt_whole, r_out, width)

# Loading & BC
ut.load_bc(mymodel, mypart, myassembly, step_name, load_name, bc_name,
           r_out, width, r_depth, r_pressure, load, s_pt_lateral)

# Job
ut.job(job_name)

# Access results
nodalS11 = ut.post_process(job_name)

# csv files for ML
ut.output_csv(mypart, results_location, nodalS11)
