#!/usr/bin/env bash

PYTHON="/Users/jensencl/Library/Application Support/Autodesk/webdeploy/production/e379c5f768f16e4cbcd81b81fe59a8562f304f19/Autodesk Fusion 360.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python"
PATH=~"/Users/jensencl/Library/Application Support/Autodesk/webdeploy/production/e379c5f768f16e4cbcd81b81fe59a8562f304f19/Autodesk Fusion 360.app/Contents/Frameworks/MAGSDK/MAGSystem.framework/Versions/A":$PATH
PYTHONHOME="/Users/jensencl/Library/Application Support/Autodesk/webdeploy/production/e379c5f768f16e4cbcd81b81fe59a8562f304f19/Autodesk Fusion 360.app/Contents/Frameworks/Python.framework/Versions/Current"
PYTHONPATH="/Users/jensencl/Library/Application Support/Autodesk/webdeploy/production/e379c5f768f16e4cbcd81b81fe59a8562f304f19/Autodesk Fusion 360.app/Contents/Frameworks/Python.framework/Versions/Current/lib/python3.7/site-packages"

# "$PYTHON"
cd .. && "$PYTHON" development_files/s3dtofusion_test.py