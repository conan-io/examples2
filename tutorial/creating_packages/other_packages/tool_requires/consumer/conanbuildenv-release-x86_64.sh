echo Capturing current environment in "/Users/luism/workspace/examples2/tutorial/creating_packages/other_packages/tool_requires/consumer/deactivate_conanbuildenv-release-x86_64.sh"
echo "echo Restoring environment" >> "/Users/luism/workspace/examples2/tutorial/creating_packages/other_packages/tool_requires/consumer/deactivate_conanbuildenv-release-x86_64.sh"
for v in MY_VAR PATH
do
    is_defined="true"
    value=$(printenv $v) || is_defined="" || true
    if [ -n "$value" ] || [ -n "$is_defined" ]
    then
        echo export "$v='$value'" >> "/Users/luism/workspace/examples2/tutorial/creating_packages/other_packages/tool_requires/consumer/deactivate_conanbuildenv-release-x86_64.sh"
    else
        echo unset $v >> "/Users/luism/workspace/examples2/tutorial/creating_packages/other_packages/tool_requires/consumer/deactivate_conanbuildenv-release-x86_64.sh"
    fi
done

echo Configuring environment variables

export MY_VAR="23"
export PATH="/Users/luism/.conan2/p/ae33adf2f113a821/p/bin:$PATH"