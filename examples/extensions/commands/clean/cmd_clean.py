from conan.api.conan_api import ConanAPI
from conan.api.model import PackagesList, ListPattern
from conan.api.input import UserInput
from conan.api.output import ConanOutput, Color
from conan.cli.command import OnceArgument, conan_command

recipe_color = Color.BRIGHT_BLUE
removed_color = Color.BRIGHT_YELLOW


@conan_command(group="Custom commands")
def clean(conan_api: ConanAPI, parser, *args):
    """
    Deletes (from local cache or remotes) all recipe and package revisions but
    the latest package revision from the latest recipe revision.
    """
    parser.add_argument('-r', '--remote', action=OnceArgument,
                        help='Will remove from the specified remote')
    parser.add_argument('--force', default=False, action='store_true',
                        help='Remove without requesting a confirmation')
    args = parser.parse_args(*args)

    def confirmation(message):
        return args.force or ui.request_boolean(message)

    ui = UserInput(non_interactive=False)
    out = ConanOutput()
    remote = conan_api.remotes.get(args.remote) if args.remote else None
    output_remote = remote or "Local cache"

    # Get all recipes and packages, where recipe revision is not the latest
    pkg_list = conan_api.list.select(ListPattern("*/*#!latest:*#*", rrev=None, prev=None), remote=remote)
    if pkg_list and not confirmation("Do you want to remove all the recipes revisions and their packages ones, "
                                    "except the latest package revision from the latest recipe one?"):
        out.writeln("Aborted")
        return

    # Remove all packages for old recipe revisions
    for recipe_ref, recipe_bundle in pkg_list.refs().items():
        conan_api.remove.recipe(recipe_ref, remote=remote)
        out.writeln(f"Removed recipe revision: {recipe_ref.repr_notime()} "
                    f"and all its package revisions [{output_remote}]", fg=removed_color)

    # Get all package revisions from the latest recipe revision, except the latest package revision
    pkg_list = conan_api.list.select(ListPattern("*/*#latest:*#!latest", rrev=None, prev=None), remote=remote)
    for recipe_ref, recipe_bundle in pkg_list.refs().items():
        pkg_list = PackagesList.prefs(recipe_ref, recipe_bundle)
        for pkg_ref in pkg_list.keys():
            # Remove all package revisions except the latest one
            conan_api.remove.package(pkg_ref, remote=remote)
            out.writeln(f"Removed package revision: {pkg_ref.repr_notime()} [{output_remote}]", fg=removed_color)