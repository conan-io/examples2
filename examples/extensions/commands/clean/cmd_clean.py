from conan.api.conan_api import ConanAPIV2
from conans.cli.command import conan_command, OnceArgument
from conans.cli.output import Color, ConanOutput
from conans.client.userio import UserInput

recipe_color = Color.BRIGHT_BLUE
removed_color = Color.BRIGHT_YELLOW


@conan_command(group="Custom commands")
def clean(conan_api: ConanAPIV2, parser, *args):
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

    # Getting all the recipes
    recipes = conan_api.search.recipes("*/*", remote=remote)
    if recipes and not confirmation("Do you want to remove all the recipes revisions and their packages ones, "
                                    "except the latest package revision from the latest recipe one?"):
        return
    for recipe in recipes:
        out.writeln(f"{str(recipe)}", fg=recipe_color)
        all_rrevs = conan_api.list.recipe_revisions(recipe, remote=remote)
        latest_rrev = all_rrevs[0] if all_rrevs else None
        for rrev in all_rrevs:
            if rrev != latest_rrev:
                conan_api.remove.recipe(rrev, remote=remote)
                out.writeln(f"Removed recipe revision: {rrev.repr_notime()} "
                            f"and all its package revisions [{output_remote}]", fg=removed_color)
            else:
                all_prevs = conan_api.search.package_revisions(f"{rrev.repr_notime()}:*#*", remote=remote)
                latest_prev = all_prevs[0] if all_prevs else None
                for prev in all_prevs:
                    if prev != latest_prev:
                        conan_api.remove.package(prev, remote=remote)
                        out.writeln(f"Removed package revision: {prev.repr_notime()} [{output_remote}]", fg=removed_color)
