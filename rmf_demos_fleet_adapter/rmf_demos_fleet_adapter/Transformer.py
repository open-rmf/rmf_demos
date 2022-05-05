# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pyproj import Transformer


class CrsTransformer:
    def __init__(self, crs):
        self.crs = crs
        # Add a base x and y to map RMF coordinates to SVY21
        self.min_x = 22000
        self.min_y = 31500

        self.svy_transformer = Transformer.from_crs(crs, 'EPSG:3414')
        self.crs_transformer = Transformer.from_crs('EPSG:3414', crs)

    def transform_rmf_to_crs(self, rmf_x, rmf_y):
        svy_x = rmf_x + self.min_x
        svy_y = rmf_y + self.min_y

        if self.crs == 'EPSG:3414':
            return svy_x, svy_y

        crs_x, crs_y = self.crs_transformer.transform(svy_x, svy_y)
        return crs_x, crs_y

    def transform_crs_to_rmf(self, crs_x, crs_y):
        if self.crs == 'EPSG:3414':
            return crs_x, crs_y

        svy_x, svy_y = self.svy_transformer.transform(crs_x, crs_y)
        rmf_x = svy_x - self.min_x
        rmf_y = svy_y - self.min_y
        return rmf_x, rmf_y
