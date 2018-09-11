#!/bin/bash
# Script to clean the tree from all compiled files.
# You can rebuild them afterwards using "build.sh"
# or build_twiddle.sh.
#
# Written by Tiffany Huang, 12/14/2016
#

# Remove the dedicated output directories
cd `dirname $0`

rm -rf build
rm -rf build_twiddle

# We're done!
echo Cleaned up the project!
