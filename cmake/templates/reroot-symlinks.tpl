#!@TOOL_BASH@
set -e
@TOOL_FIND@ $1 -type l -print | while read -r i
do
    target=$(@TOOL_READLINK@ $i)
    dir=$(@TOOL_DIRNAME@ $i)
    if [[ "$target" = /* ]] && [ "$target" != "/dev/null" ]; then
        relative=$(@TOOL_REALPATH@ --relative-to="$dir" $1$target)
        echo "Rerooting $i..."
        unlink $i
        ln -s $relative $i
    fi
done