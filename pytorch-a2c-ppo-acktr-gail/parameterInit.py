def StartMinimum():
    desiredStepDuration = 0.1
    desiredStepLength = desiredStepDuration*2/3.0
    swingfootHeight = desiredStepDuration/4.0
    return desiredStepDuration, desiredStepLength, swingfootHeight
def StartMedium():
    desiredStepDuration = 0.3
    desiredStepLength = 0.1+desiredStepDuration*2/3.0
    swingfootHeight = 0.075+desiredStepDuration/4.0
    return desiredStepDuration, desiredStepLength, swingfootHeight
