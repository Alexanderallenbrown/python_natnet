# coding: utf-8
"""Command-line NatNet client application for testing.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

from __future__ import print_function
import argparse
import time
import attr
import natnet
import math
import time

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians


@attr.s
class ClientApp(object):

    _client = attr.ib()

    # _quiet = attr.ib()
    #
    # _last_printed = attr.ib(0)

    @classmethod
    def connect(cls, server_name):

        client = natnet.Client.connect(server_name,True)
        if client is None:
            return None
        return cls(client)

    def run(self):
        # self._client.set_callback(self.callback)
        # self._client.spin()
        self.x,self.y,self.z,self.roll,self.pitch,self.yaw = 0,0,0,0,0,0

    def callback(self, rigid_bodies, markers, timing):
        """
        :type rigid_bodies: list[RigidBody]
        :type markers: list[LabelledMarker]
        :type timing: TimestampAndLatency
        """
        #print()
        #print('{:.1f}s: Received mocap frame'.format(timing.timestamp))
        if rigid_bodies:
            # print('Rigid bodies:')
            for b in rigid_bodies:
                self.roll,self.pitch,self.yaw = euler_from_quaternion(b.orientation[0],b.orientation[1],b.orientation[2],b.orientation[3])
                self.x,self.y,self.z = b.position
                # print(self.x,self.y,self.z,self.roll,self.pitch,self.yaw)

def main():
    try:
        address = '127.0.0.1'
        app = ClientApp.connect(address)
        app.run()
        app._client.set_callback(app.callback)
        while True:
            app._client.spin_once()
            print(app.x,app.y,app.z)

    except natnet.DiscoveryError as e:
        print('Error:', e)


if __name__ == '__main__':
    main()
