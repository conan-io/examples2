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

    # List all recipes revisions and all their packages revisions as well
    pkg_list = conan_api.list.select(ListPattern("*/*#*:*#*", rrev=None, prev=None), remote=remote)
    if pkg_list and not confirmation("Do you want to remove all the recipes revisions and their packages ones, "
                                    "except the latest package revision from the latest recipe one?"):
        out.writeln("Aborted")
        return

    # Split the package list into based on their recipe reference
    for sub_pkg_list in pkg_list.split():
        latest = max(sub_pkg_list.items(), key=lambda item: item[0])[0]
        out.writeln(f"Keeping recipe revision: {latest.repr_notime()} "
                    f"and its latest package revisions [{output_remote}]", fg=recipe_color)
        for rref, packages in sub_pkg_list.items():
            # For the latest recipe revision, keep the latest package revision only
            if latest == rref:
                # Get the latest package timestamp for each package_id
                latest_pref_list = [max([p for p in packages if p.package_id == pkg_id], key=lambda p: p.timestamp)
                                    for pkg_id in {p.package_id for p in packages}]
                for pref in packages:
                    if pref not in latest_pref_list:
                        conan_api.remove.package(pref, remote=remote)
                        out.writeln(f"Removed package revision: {pref.repr_notime()} [{output_remote}]", fg=removed_color)
            else:
                # Otherwise, remove all outdated recipe revisions and their packages
                conan_api.remove.recipe(rref, remote=remote)
                out.writeln(f"Removed recipe revision: {rref.repr_notime()} "
                            f"and all its package revisions [{output_remote}]", fg=removed_color)
