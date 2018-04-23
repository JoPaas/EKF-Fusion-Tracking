A brief comparison between different modes of tracking on dataset1.

RMSE lidar only
0.1474
0.1152
0.6969
0.5332

0.00927543 0 0.0243948 0
0 0.00927543 0 0.0243948
0.0243948 0 0.1486 0
0 0.0243948 0 0.1486

RMSE radar only
0.2275
0.3459
0.5985
0.8360

0.0392698 0.0205092 0.0631524 0.0298122
0.0205092 0.0230066 0.0393391 0.030205
0.0631524 0.0393391 0.21758 0.0971346
0.0298122  0.030205 0.0971346  0.131711

RMSE fusion
0.0981
0.0858
0.4659
0.4755

0.00706985 0.00226997  0.0184169 0.00695398
0.00226997 0.00511771 0.00826626 0.0114158
0.0184169 0.00826626 0.120551 0.0371732
0.00695398 0.0114158 0.0371732 0.0888411

It can be observed, that the covariance matrix of the fusion approach has smaller values, that means the filter is more certain of the state of the tracked vehicle.
The covariance matrix of the lidar only example holds several zeros, because the linear updates only result in a corellation of spped and position along one axis. The radar measurement equation is more involved, hence there are more connections between the states.