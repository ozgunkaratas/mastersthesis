def changeAxes(hlAccX,hlAccY,hlAccZ,hlGyrX,hlGyrY,hlGyrZ):
    '''assign similar axes to each other depending on the xsens frame. this does
    not guarante similar orientation, which will be done in a further step.
    '''


    #change gyroscope values in terms of XSENS

    #x-axis stays the same
    
    #y-axis
    temp = hlGyrY
    hlGyrY = hlGyrZ
    
    #z-axis
    hlGyrZ = -temp
    
    #change accelerometer values in terms of XSENS
    
    #x-axis stays the same
    
    #y-axis
    temp = hlAccY
    hlAccY = hlAccZ
    
    #z-axis
    
    hlAccZ = temp
 
    
    return(hlAccX,hlAccY,hlAccZ,hlGyrX,hlGyrY,hlGyrZ)
    