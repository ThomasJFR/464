def calibrate_z(psm):
    """
    WILL HANG THE PROGRAM
    """
    # Hang the program so the user can place the arm
    raw_input("Press enter to continue")

    # Extract arm position
    z = psm.measured_cp().p[2]
    return z
