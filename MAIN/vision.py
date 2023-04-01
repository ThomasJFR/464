import numpy as np

def capture_system():
    "Function that gets a photo of the system that we can extract features from"
    pass

def extract_system_features(image):
    """
    Extract targets and bowls from an image
    """
    # Process image to identify positions of targets and bowls

    # Dummy logic:
    features = dict()
    #psm1pos = np.array([-1.513,  -0.0819994, 05]) 
    features["items"] = [
       # np.array([ -1.523,  -0.0469994,      0.5655])
    #]
        np.array([-1.527791262, 0.05018193647, 0.674774766]),
        np.array([-1.516814113, -0.04662011936, 0.6747748256]),
        np.array([-1.547450662, -0.05359531567, 0.674774766]),
        np.array([-1.536615372, 0.003250310896, 0.674774766]),
        np.array([-1.51279664, 0.02789815143, 0.674774766]),
        np.array([-1.550005555, -0.02500112727, 0.6747747064]),
    ]

    for i in range(len(features["items"])):
        features["items"][i][2] = 0.5645
    #features["items"] = [x -  psm1pos for x in features["items"]]
    features["bowls"] = [
       # np.array([-1.443, -0.0020, 0.5655]),
        np.array([-1.473, -0.0570, 0.5655]) #- psm1pos,
    ]
    return features

