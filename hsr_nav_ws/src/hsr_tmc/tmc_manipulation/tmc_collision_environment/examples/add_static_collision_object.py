#!/usr/bin/env python3
'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
# -*- coding: utf-8 -*-
import itertools
import os

from tmc_collision_environment.collision_object_factory import (
    generate_object_from_mesh_file,
    generate_shelf,
    generate_spheres,
    generate_table_box,
)
from tmc_collision_environment.collision_object_register import main


_ORIGIN_FRAME_ID = 'odom'


if __name__ == '__main__':
    main([generate_object_from_mesh_file('cone', os.path.join(os.path.dirname(__file__), 'cone.stl'),
                                         0.0, 1.0, -0.2, ref_frame_id=_ORIGIN_FRAME_ID),
          generate_table_box('table', [0.4, 0.5, 0.4], 1.0, 0.5, 0.0, _ORIGIN_FRAME_ID),
          generate_shelf('shelf', [0.28, 0.80, 1.06], [0.11, 0.40, 0.75], 0.75, -0.5, 3.14, 0.02, _ORIGIN_FRAME_ID),
          generate_spheres('points', [[x, y, 0.0] for x, y in itertools.product([1.0, 1.1, 1.2], repeat=2)],
                           ref_frame_id=_ORIGIN_FRAME_ID)])
