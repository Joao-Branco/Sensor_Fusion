import itertools
import target_dynamics
import math

# # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # #
############CONDITIONS#################
# # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # #

conditions = []
# TRUE to be removed
# FALSE to keep
conditions.append(lambda iteration: not (iteration[15] > 0 and iteration[12] != 'augmented_state')) # AUG = 0 WHEN DELAY STRATEGY IS NOT AUGMENTED STATE
conditions.append(lambda iteration: not (iteration[16] < 1 and iteration[17] is True)) # WHEN PI  IS NOT 1 IT IS ALWAYS DISTRIBUTED
conditions.append(lambda iteration: not (iteration[2] == 1 and iteration[10] is True)) # WHEN ONLY 1 UAV SHARE IS FALSE
conditions.append(lambda iteration: not (iteration[12] == 'augmented_state' and not (
                                         iteration[15] >= math.floor(iteration[14][0] / (1 / iteration[4])) + math.floor(iteration[14][1] / (1 / iteration[4]))
                                         and iteration[15] - (math.floor(iteration[14][0] / (1 / iteration[4])) + math.floor(iteration[14][1] / (1 / iteration[4]))) <= 1))) # WHEN NUMBER OF AUG SMALL TO CORRECT THE DELAY AND WHEN ITS TOO BIG TO CORRECT THE DELAY
#ONLY ONE OPTION OF AUG FOR EACH SET OF DELAY
conditions.append(lambda iteration: not (iteration[6] == 0 and iteration[10] is True)) # WHEN share freq equals 0 and share on
conditions.append(lambda iteration: not (iteration[6] > 0 and iteration[10] is False)) # WHEN share freq exists but doenst share on

combined_condition = lambda iteration: all(cond(iteration) for cond in conditions)


def simulations(printt = [False], sim_time = [60],  n_uavs = [1], f_sim = [200], f_kf = [20], f_sample = [10], f_share = [0] , sensor = [(0, 2)],
                ekf = [True], dynamic = [target_dynamics.circular_path], share = [False], out_of_order = [False], 
                delay_strategy = [None], delay_d = [False], delay = [(0, 0)],
                AUG = [0], PI = [1], Centre = [True]
                ):
 
        if ('augmented_state' in delay_strategy):
                AUG = [0]
                for (dm, ds) in delay:
                        AUG.append(math.floor(dm / (1 / f_kf[0])) + math.floor(ds / (1 / f_kf[0])))

        for f in f_share:
                if (f > 0 and True not in share):
                        share.append(True)


        lists = [printt, sim_time, n_uavs, f_sim, f_kf, f_sample, f_share, sensor,
                ekf, dynamic, share, out_of_order,
                delay_strategy, delay_d, delay,
                AUG, PI, Centre]
        
        new_lists = []
        for lst in lists:
                new_lists.append(list(set(lst)))
        #removing duplicates in lists

    


        simulations = list(itertools.product(*new_lists))

        # CHECK CONDITIONS
        # for iteration in simulations:
        #         print(tuple(iteration))
        #         print(iteration[6])
        #         print(iteration[10])
        #         condition_results = []
        #         for cond_index, cond in enumerate(conditions):
        #                 result = cond(iteration)
        #                 condition_results.append((result, cond_index))
        #                 print(f"Condition {cond_index}: {result}")
        #         if all(result for result, _ in condition_results):
        #                 print("Combination meets all conditions.")
        #         else:
        #                 print("Combination does not meet all conditions.")

        filtered_iterations = [iteration for iteration in simulations if combined_condition(iteration)]



        return filtered_iterations




