define crazybob position
(

size [0.625 0.5 0.5]

# centre of rotation

origin [0.125 0 0 0]

# the shape of the robot

block

(

points 6

point[5] [0 0]

point[4] [0 1]

point[3] [0.75 1]

point[2] [1 0.75]

point[1] [1 0.25]

point[0] [0.75 0]

z [0 1]

)

block

(

points 4

point[3] [1 0.75]

point[2] [1.25 0.75]

point[1] [1.25 0.625]

point[0] [1 0.625]

z [0 0.5]

)

block

(

points 4

point[3] [1 0.375]

point[2] [1.25 0.375]

point[1] [1.25 0.25]

point[0] [1 0.25]

z [0 0.5]

)

# positonal things

drive "diff"
)