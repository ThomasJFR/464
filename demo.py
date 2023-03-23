from SafePSM import SafePSM1, SafePSM2, PSMController

psm1 = SafePSM1()
psm2 = SafePSM2()
psmc = PSMController(psm1, psm2)

# 0. Calibrate the PSM Controller
psm1.safe_home()
psm2.safe_home()
psmc.init()

def wave():
    psm1.safe_move_dpos(dx=-0.05)
    psm2.safe_move_dpos(dx= 0.05).wait()

    psm1.safe_move_dpos(dx= 0.1)
    psm2.safe_move_dpos(dx=-0.1).wait()

    psm1.safe_move_dpos(dx=-0.05)
    psm2.safe_move_dpos(dx= 0.05).wait()

# 1. Move the PSM controllers together
psm1.safe_move_dpos(dy= 0.05, dz=-0.08)
psm2.safe_move_dpos(dy=-0.05, dz=-0.08).wait()
wave()

# 2. Close in onto the bounding plane
psmc.update()
psm1.safe_move_pos(y= 1)
psm2.safe_move_pos(y=-1).wait()
wave()


# 3. Update the bounding plane
psmc.update(bpy=-0.04)
psm1.safe_move_dpos().wait()  # This arm should move
psm2.safe_move_dpos().wait()  # This arm should not
wave()

