#!/bin/sh

/solution/target/release/app "$@"

RETVAL=$?

if [ $RETVAL -ne 0 ]
then
  echo "run error code: $RETVAL"
  exit $RETVAL
else
  exit 0
fi
