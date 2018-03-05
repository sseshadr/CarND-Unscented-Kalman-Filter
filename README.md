# CarND-Unscented-Kalman-Filter
Use Unscented Kalman filter principles with the CTRV model to fuse LiDAR and RADAR sensor data to provide a more accurate estimate of a simulated vehicle's position and velocity.

#### NOTE: This submission depends on the Udacity visualization and data generation utilities from the utilities repository [here](https://github.com/udacity/CarND-Mercedes-SF-Utilities).

---

[//]: # (Image References)

[image1]: ./readme_images/ctrv_prediction.PNG "Prediction CTRV model test"

[image2]: ./readme_images/update_lidar.PNG "LIDAR only Tracking"

[image3]: ./readme_images/lidarnis.PNG "LIDAR NIS"

[image4]: ./readme_images/radarOnly.PNG "RADAR Only Tracking"

[image5]: ./readme_images/radarnis.PNG "RADAR NIS"

[image6]: ./readme_images/update_both.PNG "Final Tracking with both LIDAR and RADAR"

[image7]: ./readme_images/allnis.PNG "Final NIS"

---

#### Project Notes

##### Initializing the State Vector
The state vector position is initialized using the radar/lidar sensor measurements. The velocity states and the yaw are an initial guess. This guess has an effect on the overall accuracy of the filter. We can tune this to get the filter to robustly perform for different datasets.

##### Prediction
We can look at just the prediction to assure that the process follows the CTRV model by turning off the update steps. It does as shown below:

![alt text][image1]

##### Update
We can test the LIDAR and the RADAR update steps separately and use NIS to tune the process noise parameters as shown below:

![alt text][image2]

![alt text][image3]

![alt text][image4]

![alt text][image5]

Finally we can see the update for both the measurements. The RMSE for this is better than each of the measurements separately.

![alt text][image6]

![alt text][image7]

##### Normalizing Angles
This had a huge effect on the result of the tracking. The trick was to NOT normalize in the prediction step so that the sigma points are smooth. More on this in the forum post https://discussions.udacity.com/t/help-reducing-rmse/620283.

##### Avoid Divide by Zero throughout the Implementation
We can write a simple check to see if denominators don't result in 0. This is built into the sigma point prediction to switch calculations based on when the yaw derivative is 0.

##### Test Your Implementation
Testing this implementation on both datasets 1 and 2, we are able to pass the rubric check. The px, py, vx, and vy RMSE is less than or equal to the values  [.09, .10, .40, .30].

Other links that helped:
https://discussions.udacity.com/t/simulator-freezes-stuck-in-while-loop-for-angle-normalization/349245/4
https://discussions.udacity.com/t/very-high-rmse-and-x-values/343911/3
https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449/7
