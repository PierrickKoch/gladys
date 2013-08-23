import time
import gladys
f_robot  = '/home/pkoch/sandbox/gladys/tmp/robot.json'
#f_region = '/home/pkoch/sandbox/gladys/tmp/maze.region.200.tif'
f_region = '/home/pkoch/sandbox/gladys/tmp/caylusRessacLidar-0.5.dtm.region.tif'
time_1 = time.time()
g = gladys.nav_graph(f_region, f_robot)
time_2 = time.time()
g.save('test.tif')
time_3 = time.time()
g.search((1.,2.),(3.,4.))
time_4 = time.time()

print("time to load costamp and graph: %f s"% (time_2 - time_1) )
print("time to save the costamp: %f s"% (time_3 - time_2) )
print("time to search a path: %f s"% (time_4 - time_3) )

