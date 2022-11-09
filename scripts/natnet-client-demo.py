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
    _quiet = attr.ib()

    _last_printed = attr.ib(0)

    @classmethod
    def connect(cls, server_name, rate, quiet):
        if server_name == 'fake':
            client = natnet.fakes.SingleFrameFakeClient.fake_connect(rate=rate)
        else:
            client = natnet.Client.connect(server_name)
        if client is None:
            return None
        return cls(client, quiet)

    def run(self):
        if self._quiet:
            self._client.set_callback(self.callback_quiet)
        else:
            self._client.set_callback(self.callback)
        self._client.spin()

    def callback(self, rigid_bodies, markers, timing):
        """

        :type rigid_bodies: list[RigidBody]
        :type markers: list[LabelledMarker]
        :type timing: TimestampAndLatency
        """
        #print()
        #print('{:.1f}s: Received mocap frame'.format(timing.timestamp))
        if rigid_bodies:
            print('Rigid bodies:')
            for b in rigid_bodies:
                roll,pitch,yaw = euler_from_quaternion(b.orientation[0],b.orientation[1],b.orientation[2],b.orientation[3])
                print('\t Id {}: ({: 5.2f}, {: 5.2f}, {: 5.2f}), ({: 5.2f}, {: 5.2f}, {: 5.2f})'.format(
                    b.id_, *(b.position +(roll,pitch,yaw))
                ))
        # if markers:
        #     print('Markers')
        #     for m in markers:
        #         print('\t Model {} marker {}: size {:.4f}mm, pos ({: 5.2f}, {: 5.2f}, {: 5.2f}), '.format(
        #             m.model_id, m.marker_id, 1000*m.size, *m.position
        #         ))
        # print('\t Latency: {:.1f}ms (system {:.1f}ms, transit {:.1f}ms, processing {:.2f}ms)'.format(
        #     1000*timing.latency, 1000*timing.system_latency, 1000*timing.transit_latency,
        #     1000*timing.processing_latency
        # ))

    def callback_quiet(self, *_):
        if time.time() - self._last_printed > 1:
            print('.')
            self._last_printed = time.time()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', help='Will autodiscover if not supplied')
    parser.add_argument('--fake', action='store_true',
                        help='Produce fake data at `rate` instead of connecting to actual server')
    parser.add_argument('--rate', type=float, default=10,
                        help='Rate at which to produce fake data (Hz)')
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    try:
        app = ClientApp.connect('fake' if args.fake else args.server, args.rate, args.quiet)
        app.run()
    except natnet.DiscoveryError as e:
        print('Error:', e)


if __name__ == '__main__':
    main()
