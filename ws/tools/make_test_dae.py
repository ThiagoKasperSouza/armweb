#!/usr/bin/env python3
"""Build synthetic COLLADA files that reproduce the FR3 collapse.

Two files are written:

  identity.dae  every geometry sits at the origin with an identity node
                transform -- the "no node transforms" case.
  nested.dae    one parent node with a real translation, a child node with
                the geometry under it. This is the case where ignoring the
                scene graph leaves everything stacked at the origin.

run_node_walk.py then checks that load_mesh honours the transform in the
second file and is a no-op on the first.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

IDENTITY_DAE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Y_UP</up_axis></asset>
  <library_geometries>
    <geometry id="g0" name="box">
      <mesh>
        <source id="ps">
          <float_array id="pa" count="24">
            0 0 0   1 0 0   1 1 0   0 1 0
            0 0 1   1 0 1   1 1 1   0 1 1
          </float_array>
          <technique_common>
            <accessor source="#pa" count="8" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="v"><input semantic="POSITION" source="#ps"/></vertices>
        <triangles count="12" material="mat">
          <input semantic="VERTEX" source="#v" offset="0"/>
          <p>0 1 2 0 2 3 4 5 6 4 6 7 0 4 7 0 7 3
             1 5 6 1 6 2 3 7 6 3 6 2 0 1 5 0 5 4</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene" name="scene">
      <node id="n0" name="n0" type="NODE">
        <instance_geometry url="#g0"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""

# The geometry is a unit box; the node hierarchy moves it to (10, 0, 0).
NESTED_DAE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Y_UP</up_axis></asset>
  <library_geometries>
    <geometry id="g0" name="box">
      <mesh>
        <source id="ps">
          <float_array id="pa" count="24">
            0 0 0   1 0 0   1 1 0   0 1 0
            0 0 1   1 0 1   1 1 1   0 1 1
          </float_array>
          <technique_common>
            <accessor source="#pa" count="8" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="v"><input semantic="POSITION" source="#ps"/></vertices>
        <triangles count="12" material="mat">
          <input semantic="VERTEX" source="#v" offset="0"/>
          <p>0 1 2 0 2 3 4 5 6 4 6 7 0 4 7 0 7 3
             1 5 6 1 6 2 3 7 6 3 6 2 0 1 5 0 5 4</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene" name="scene">
      <node id="parent" name="parent" type="NODE">
        <translate sid="t">10 0 0</translate>
        <node id="child" name="child" type="NODE">
          <matrix sid="m">1 0 0 0  0 1 0 0  0 0 1 0  0 0 0 1</matrix>
          <instance_geometry url="#g0"/>
        </node>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""


def main():
    for name, body in (("identity.dae", IDENTITY_DAE),
                       ("nested.dae", NESTED_DAE)):
        p = os.path.join(HERE, name)
        with open(p, "w", encoding="utf-8") as fh:
            fh.write(body)
        print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())