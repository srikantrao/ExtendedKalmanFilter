# Extended Kalman Filter

## Eclipse IDE was used, steps followed to build are listed below - 

1. mkdir build
2. cd build
3. cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../src/
4. make
5. ./ExtendedKF

### Result for dataset 1 - 
![alt text](https://github.com/srikantrao/ExtendedKalmanFilter/blob/master/dataset1.png "Result for DataSet 1")

### Result for dataset 2 - 
![alt text](https://github.com/srikantrao/ExtendedKalmanFilter/blob/master/dataset2.png "Result for DataSet 2")

### Expected RMSE values for [px,py,vx,vy] are less than or equal to the values [.11, .11, 0.52, 0.52].
