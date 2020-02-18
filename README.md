# Muscle-Model

Here you can find the details on one of my arm models. This model was developed with directly minimizing the cost with respect to the constrains. The cost is minimizing the rate of force in the 6 set of muscles. 

Dynamics: Twolink arms with 6 set of muscles

States: X = [x, y, dx, dy, f1, f2, ..., f6]

Command: u = [df1, df2,...df6] --> the rate of muscle force

Cost: Square rate of force in the muscles J = u^2

Constrains: starting and ending point / zero initial and final velocity

Optimizer: algorithm = 'active-set';
