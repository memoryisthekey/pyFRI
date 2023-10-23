#!/usr/bin/env python

import numpy as np
from spatialmath.base import trotz, transl
from roboticstoolbox import DHRobot, RevoluteMDH, RevoluteDH


class Med7(DHRobot):

    def __init__(self):
        gear_ratio = 1.0

        L = [RevoluteMDH(d=0.340, a=0, alpha=0, offset=0,
                            qlim=[-2.9671, 2.9671],
                            m = 3.4525,
                            I = [0.02183, 0, 0, 0.007703, -0.003887, 0.02083] ,
                            G = gear_ratio,
                            #r = [0, -0.03, 0.12]                           
                        ),
             RevoluteMDH(d=0.0, a=0, alpha=-np.pi/2, offset=0,
                            qlim=[-2.0944, 2.0944],
                            m = 3.4821,
                            I = [0.02076, 0, -0.003626, 0.02179, 0, 0.00799],
                            G = gear_ratio,
                            #r = [0.0003, 0.059, 0.042]    
                        ),
             RevoluteMDH(d=0.400, a=0, alpha=np.pi/2, offset=0,
                            qlim=[-2.9671, 2.9671],
                            m = 4.05623,
                            I = [0.03204, 0, 0, 0.00972, 0.0062227, 0.03042],
                            G = gear_ratio,
                            #r = [0, 0.03, 0.13]
                        ),
             RevoluteMDH(d=0.0, a=0, alpha=np.pi/2, offset=0,
                            qlim=[-2.0944, 2.0944],
                            m = 3.4822,
                            I = [0.02178, 0, 0, 0.02075, -0.003625, 0.007785],
                            G = gear_ratio,
                            #r = [0, 0.067, 0.034]
                        ),
             RevoluteMDH(d=0.400, a=0, alpha=-np.pi/2, offset=0,
                            qlim=[-2.9671, 2.9671],
                            m = 2.1633,
                            I = [0.01287, 0, 0, 0.005708, -0.003946, 0.01112],
                            G = gear_ratio,
                            #r = [0.0001, 0.021, 0.076]
                        ),
             RevoluteMDH(d=0.0, a=0, alpha=-np.pi/2, offset=0,
                            qlim=[-2.0944, 2.0944],
                            m = 2.3466,
                            I = [0.006509, 0, 0, 0.006259, 0.00031891, 0.004527],
                            G = gear_ratio,
                            #r = [0, 0.0006, 0.0004]
                        ),
             RevoluteMDH(d=0.126, a=0, alpha=np.pi/2, offset=0,
                            qlim=[-3.0543, 3.0543],
                            m = 3.129,
                            I = [0.01464, 0.0005912, 0, 0.01465, 0, 0.002872],
                            G = gear_ratio,
                            #r = [0, 0, 0.02]                        
                        )]
        
        tool = transl(0, 0, 0.190)
        super().__init__(
            L,
            name="med7",
            manufacturer="kuka",
            meshdir="",
            tool=tool,
        )

        self.home = np.zeros(7)

        self.addconfiguration("home", self.home)

   

#if __name__ == "__main__":  # pragma nocover
#
#    med7 = Med7()
#    q1 = np.zeros(7)
#    q2 = np.zeros(7) + np.array([np.pi, np.pi, 1, 0.5, 0.5, 0.1, 0.4])
#    # only print 3 decimals numpy
#    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)
#    print(med7.fkine(q1))
#    print(med7.fkine(q2))
#    print(med7.jacobe(q2))
#    print(med7.pybot.jacobian_flange(q2))
#    
#    #print(med7.pybot.jacobian_world(q2))
  