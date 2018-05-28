# Project Write-up: 3D Quad Estimation using EKF

---
#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!
Here I will consider the [rubric](https://review.udacity.com/#!/rubrics/1807/view)
points individually and describe how I addressed each point in my implementation.

### Implement Estimator

#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

After running `Scenario 6: Noise Sensors` we find files
- config/log/Graph1.txt
- config/log/Graph2.txt

containing logged data from GPS and accelerometer that looks like this:
```
time, Quad.GPS.X
0.105000,0.929727
0.205000,-0.850977
```

Using simple `python` code we can calculate standard deviations from that data:

```
>>> import numpy as np
>>> g1 = np.genfromtxt('config/log/Graph1.txt',delimiter=',',skip_header=1)
>>> print("GPS.X std = {}".format(np.std(g1[:,1])))
GPS.X std = 0.598544329703513
>>> g2 = np.genfromtxt('config/log/Graph2.txt',delimiter=',',skip_header=1)
>>> print("Accelerometer.X std = {}".format(np.std(g2[:,1])))
Accelerometer.X std = 0.5115638316431003
```

When plugging the data into `06_SensorNoise.txt` we see that empirically ~68% of observations fall
within found estimates of standard deviation:

![Scenario 6](./images/writeup/std.png)

#### 2. Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.

Requrements:
The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.







Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd







