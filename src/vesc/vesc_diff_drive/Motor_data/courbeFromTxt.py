import numpy as np
import os
import matplotlib.pyplot as plt


# Input folder
#folder_path 

# Opening file
file = open("Motor_test.txt","r") 

# Reading the file
content = file.read()

file.close()

# Splitting each list of number first by \n then by coma
a = content.split("\n")
left_speeds = map(float, (a[1].replace(" ","")).split(","))
timeL = map(float, (a[3].replace(" ","")).split(","))
right_speeds = map(float, (a[5].replace(" ","")).split(","))
timeR = map(float, (a[7].replace(" ","")).split(","))



diff_time_left=[]
for i in range(1, len(timeL)):
	diff_time_left.append(timeL[i] - timeL[i-1])

diff_time_right=[]
for i in range(1, len(timeR)):
	diff_time_right.append(timeR[i] - timeR[i-1])

diff_time = []

for i in range(max(len(timeL), len(timeR))):
	diff_time.append(timeL[i] - timeR[i])

speed_diff=[]
for i in range(max(len(left_speeds ), len(right_speeds ))):
	speed_diff.append(left_speeds[i] - right_speeds[i])


sum_distance_left=[]
sum_distance_right=[]
sum_distance_left.append(0)
sum_distance_right.append(0)
for i in range(1,len(left_speeds)):
	#print("left_speed:"+str(left_speeds[i]))
	#print("right_speed:"+str(right_speeds[i]))
	sum_distance_left.append( ((left_speeds[i]*(timeL[i]-timeL[i-1])*0.1)/6.0) + sum_distance_left[i-1]  )
	sum_distance_right.append( ((right_speeds[i]*(timeR[i]-timeR[i-1])*0.1)/6.0) + sum_distance_right[i-1] )



########################################################
##      Evolution de la difference de vitesse )   
########################################################

plt.xlabel('index of time')
plt.ylabel('Difference de distance en m')
plt.title('Evolution de la difference de vitesse ')


axis=np.linspace(0, len(speed_diff)-1,len(speed_diff) ) #len(timeR))
plt.plot(axis, speed_diff, label='Evolution de la difference de vitesse ')

plt.legend()
plt.savefig('Evolution_de_la_difference_de_vitesse.png')
plt.clf()


########################################################
##      Evolution des distances parcourus gauche et droite 
########################################################

plt.xlabel('index of time')
plt.ylabel('Distance gauche et droite parcouru en m')
plt.title('Evolution des distances parcourus gauche et droite ')

plt.plot(timeL, sum_distance_left, label='left distance')
plt.plot(timeR, sum_distance_right, label='right distance')


plt.legend()
plt.savefig('Evolution_des_distances_parcourus_gauche_et_droite.png')
plt.clf()


########################################################
##      Evolution du flux du temps  
########################################################

plt.xlabel('Time flux')
plt.ylabel('Flux of time')
plt.title('index')

length_of_plot=10
axis=np.linspace(0,length_of_plot-1,length_of_plot) #len(timeR))
plt.plot(axis,timeL[0:length_of_plot],label='left time')
plt.plot(axis,timeR[0:length_of_plot],label = 'right time')
#plt.plot(axis,axis, label = 'right time')

plt.legend()
plt.savefig('Evolution_of_time_flux.png')
plt.clf()

########################################################
##      Evolution de la vitesse    
########################################################


plt.xlabel('Time in s')
plt.ylabel('Speed in rad/s')
plt.title('Evolution of speed')

plt.plot(timeL, left_speeds,label='left speed')
plt.plot(timeR, right_speeds, label = 'right speed')

plt.legend()
plt.savefig('Evolution_of_speed.png')
plt.clf()


########################################################
##      Difference of time   
########################################################


axis_bis=np.linspace(0,len(diff_time_left)-1,len(diff_time_left)) #len(timeR))


plt.xlabel('index')
plt.ylabel('Difference of time s')
plt.title('Evolution of Difference of time ')

plt.plot(axis_bis, diff_time_left ,label='left  difference of time')
plt.plot(axis_bis, diff_time_right, label = 'right  difference of time')

plt.legend()
plt.savefig('Evolution of Difference of time.png')
plt.clf()

########################################################
##      Difference of time   between Left and right
########################################################

plt.xlabel('Time in s')
plt.ylabel('Difference of time in s')
plt.title('Difference of time')

plt.plot(range(len(diff_time)), diff_time)

plt.savefig('difference_of_time.png')




