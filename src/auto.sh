#!/usr/bin/bash
source ../../../devel/setup.sh
if [ ! -d "proto/python_out" ]; then
    mkdir -p proto/python_out
fi
for file2 in `ls -a proto`
    do
        if [ x"$file2" != x"." -a x"$file2" != x".." -a x"$file2" != x"__init__.py" ];then
            if [ ! -d "proto/$file2" ]; then
                if [ ! -d "../../data-relay/proto/${file2%.proto}" ]; then
                    mkdir ../../data-relay/proto/${file2%.proto}
                fi
                protoc -I=proto --go_out=paths=source_relative:../../data-relay/proto/${file2%.proto} proto/${file2}
                protoc -I=proto --python_out=proto/python_out proto/${file2}
            fi
        fi
done

cd proto/python_out
sed -i -E 's/^import.*_pb2/from . \0/' *.py
cd ../../
