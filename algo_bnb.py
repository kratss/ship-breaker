best_cost = np.inf
c_idx = 0  # current node index
c_cost = 0 # current cost

unvisited_nodes = nodes - node[c_idx]
for i in range unvisited_nodes:
    # expand node i 
    if cost[idx,i] > best_cost:
        continue 
    else:
        c_cost +=- cost[idx,i]

