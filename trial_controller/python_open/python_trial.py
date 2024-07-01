import sys

sys.path.insert(1, '/home/saima/eugenio/panda_robot/catkin_ws/my_optimizers/rosenbrock')
import rosenbrock

solver = rosenbrock.solver()
result = solver.run(p=[20., 1.])
u_star = result.solution

print(u_star)
