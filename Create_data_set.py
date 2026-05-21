#import cplex
#from numpy import genfromtxt
#from cplex.exceptions import CplexError
import csv
import numpy as np
import random
import math
import time

def main ():
    data_name = '01001'   #first 3 digits are number of customers. last 2 digits represent version of data set from 1-99.
    total_nodes = 10        #doesn't count depot
    truck_speed = 25/60     
    drone_speed = 35/60
    cannot_use_d = round(total_customers*0.15)    #portion of customers who cannot receive services by the drone
    max_xy = 8         #size of x-y plane. If you want more customers, you have to scale this parameter to maintain the customer density

    #### create nodes solution
    save_solution = np.zeros((int(total_customers+2),4))
    for i in range(0,total_customers+1):
        save_solution[i][0] = i
        save_solution[i][1] = round(random.uniform(0,max_xy),2)
        save_solution[i][2] = round(random.uniform(0,max_xy),2)

    #for the depot
    save_solution[total_customers+1][0] = total_customers + 1
    save_solution[total_customers+1][1] = save_solution[0][1]
    save_solution[total_customers+1][2] = save_solution[0][2]

    used_number = []
    for i in range(1,cannot_use_d+1):
        c = random.randint(1, total_customers)
        while c in used_number:
            c = random.randint(1, total_customers)
        save_solution[c][3] = 1
        used_number.append(c)
    print(save_solution)

    ###### create file for c prime
    c_prime = []
    for i in range(0,total_customers+2):
        c_prime.append(i)

    c_prime_not = []
    for i in range(0,total_customers + 2):
        if save_solution[i][3] == 1:
            c_prime_not.append(i)

    for i in range(0, len(c_prime_not)):
        c_prime.remove(c_prime_not[i])
    c_prime.remove(int(0))
    c_prime.remove(int(total_customers+1))


    with open('Cprime' + '_' + data_name + '.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(c_prime)

    #### create nodes file
    with open('nodes' + '_' + data_name + '.csv', 'w',newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(save_solution)


    #### for time matrix of truck (\tau) manhattan
    time_truck = np.zeros((int(total_customers + 2), (int(total_customers + 2))))
    for i in range(0,total_customers+2):
        for j in range(0,total_customers+2):
            a = abs(save_solution[i][1] - save_solution[j][1])
            b = abs(save_solution[i][2] - save_solution[j][2])
            time_truck[i][j] = (a+b)/truck_speed

    ### for time matrix of drone (\tau_prime) unqliduen
    time_drone = np.zeros((int(total_customers + 2), (int(total_customers + 2))))
    for i in range(0,total_customers+2):
        for j in range(0,total_customers+2):
            a = abs(save_solution[i][1] - save_solution[j][1])
            b = abs(save_solution[i][2] - save_solution[j][2])
            c = math.sqrt((a*a) + (b*b))
            time_drone[i][j] = c/drone_speed

    ####Create files for time
    print("truck distance")
    with open('tau' + '_' + data_name + '.csv', 'w',newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(time_truck)
    print(time_truck)
    print("drone distance")
    with open('tauprime' + '_' + data_name + '.csv', 'w',newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(time_drone)
    print(time_drone)

main()