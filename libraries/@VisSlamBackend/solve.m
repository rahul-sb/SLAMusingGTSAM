function solve(obj)

import gtsam.*

optimizer = DoglegOptimizer(obj.graph, obj.initial_estimate);
obj.result = optimizer.optimize();

end