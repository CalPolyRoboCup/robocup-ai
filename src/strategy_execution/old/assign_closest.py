import numpy as np
import copy
import itertools

# order points by importance
# an approximation
def assign_closest(points, actors):
  res = []
  for p in points:
    if len(res) == len(points):
      break
    best_dist = 0
    best_ind = -1
    ind = 0
    for a in actors:
      if ind in res:
        ind += 1
        continue
      dist = np.linalg.norm(a.loc - points)
      if best_ind == -1 or dist < best_dist:
        best_dist = dist
        best_ind = ind
      ind += 1
    res.append(best_ind)
  return res

#I tried an exhaustive search and had problems
#I'm using it for vectorized computation not Neural Nets
# import tensorflow as tf

# tf.enable_eager_execution()
# tf.train.get_or_create_global_step()

# def myperm(points, length, depth = 0):
  # if depth == length:
    # return []
  # res = []
  # for i in range(len(points)):
    # nres = []
    # prem = copy.deepcopy(points)
    # prem.pop(i)
    # for p in myperm(prem, length, depth + 1):
      # res.append()
  # return res

# #this function scales O(N!) do not make points array large
# def assign_closest(points, actors, priorities = 1):
  # if len(points) < len(actors):
    # points.extend([-1 for i in range(len(actors) - len(points))])
  # print(15, points)
  # permutations = myperm(points, len(actors))
  # print(permutations)
  # permutations = tf.constant(permutations, dtype = tf.float32)
  # alocs = [a.loc for a in actors]
  # print(tf.norm(permutations - alocs) * priorities)
  # values = -tf.reduce_mean(tf.norm(permutations - alocs) * priorities, axis = -1)
  # _, ind = tf.math.top_k(values)
  # return permutations[ind]
  

#TODO: Fix this function, it is a hot mess
#      I think it is good enough to use in the short term, but it needs work
#      It is reasonably fast for small inputs and gives solutions that 
#      are close to correct.
# def assign_closest(points, actors):
  # distance_matrix = []
  # dm_index = []
  # selected = []
  # a_ind = 0
  # for a in actors:
    # distances = []
    # ind = 0
    # for p in points:
    
    
      # # TODO: Should take current velocities into account
      
      
      # distances.append((np.linalg.norm(a.loc - p), ind))
      # ind += 1
    # # sort points by distances
    # distances.sort(key = lambda val : val[0])
    # distance_matrix.append(distances)
    # # print(distance_matrix)
    # # print(selected)
    # # print(distances)
    # # print(dm_index)
    # #print("new actor", selected, dm_index)
    # gp, selected, dm_index = glom_points(distance_matrix, selected, distances, dm_index, a_ind)
    # if gp == -1:
      # selected.append(-1)
    # a_ind += 1
  # return selected
# def glom_points(distance_matrix, selected, distances, dm_index, ind, i = 0):
  # #print("s", selected)
  # while (i < len(distances)):
    # #print("sss", selected)
    # gp, selected, dm_index = glom_point(distance_matrix, selected, distances, dm_index, ind, i)
    # #print("ssss", selected)
    # #print("result", gp, dm_index)
    # if -1 != gp:
      # return gp, selected, dm_index
    # i += 1
  # return -1, selected, dm_index
# def glom_point(distance_matrix, selected, distances, dm_index, ind, i):
  # #input()
  # if i == len(distances):
    # return -1, -1
  # #print("ss", selected, distances[i][1])
  # s = 0
  # found = False
  # while s < len(selected):
    # #print(s, selected[s], distances[i][1])
    # if selected[s] == distances[i][1]:
      # found = True
      # break
    # s += 1
  # #if the best point isn't taken take it
  # if not found:
    # #print("new point", distances[i][1], selected)
    # if ind == len(selected):
      # dm_index.append(i)
      # selected.append(distances[i][1])
    # else:
      # dm_index[ind] = i
      # selected[ind] = distances[i][1]
      # #print("rewrite", ind, len(selected), selected, distances[i][1])
    # return distances[i][0], selected, dm_index
  # #otherwise if we can make the swap swap it
  # elif i != len(distances) - 1:
    # #print("cs", distances[i][1], s, distance_matrix.index(distances), ind, dm_index)
    # #print("check swap", immediate_gain, distances[i+1][0] - distances[i][0], (swap_glom[0] - distance_matrix[s][dm_index[s]][0]))
    # immediate_gain = (distances[i + 1][0] - distances[i][0]) - (distance_matrix[s][dm_index[s] + 1][0] - distance_matrix[s][dm_index[s]][0])
    # if immediate_gain > 0:
      # copy_select = copy.deepcopy(selected)
      # #copy_select.remove(distances[i][1])
      # copy_dm_index = copy.deepcopy(dm_index)
      # copy_dm_index[s] += 1
      # if ind == len(copy_dm_index):
        # #print("cmd dm", ind, i, copy_dm_index)
        # copy_dm_index.append(i)
      # #print("glom", copy_dm_index, distance_matrix.index(distances))
      # swap_glom, copy_select, copy_dm_index = glom_points(distance_matrix, copy_select, distance_matrix[s], copy_dm_index, s, dm_index[s] + 1)
      # #print("glom", copy_dm_index, distance_matrix.index(distances))
      # if swap_glom > 0 and immediate_gain - swap_glom + distance_matrix[s][dm_index[s]][0] > 0:
        # #print("swap_successful", immediate_gain - swap_glom + distance_matrix[s][dm_index[s]][0], s, distance_matrix.index(distances))
        # #dm_index = copy_dm_index
        # #dm_index.append(i)
        # #copy_select.append(distances[i][1])
        # #print(80,copy_select, selected, distance_matrix.index(distances))
        # #selected = copy_select
        # #print(82, copy_select, selected, distance_matrix.index(distances))
        # if ind == len(selected):
          # copy_select.append(distances[i][1])
        # else:
          # copy_select[ind] = distances[i][1]
        # #print(copy_select, copy_dm_index)
        # return immediate_gain - swap_glom + distance_matrix[s][dm_index[s]][0], copy_select, copy_dm_index
  # return -1, selected, dm_index
   
   
class proxy_bot:
  def __init__(self, loc):
    self.loc = np.array(loc)
   
if __name__ == "__main__":
  actors = [proxy_bot([x, 0]) for x in range(10)]
  targets = [np.array([x, 5]) for x in range(5)]
  targets.extend([np.array([x+1, 2]) for x in range(3)])
  print(assign_closest(targets, actors))