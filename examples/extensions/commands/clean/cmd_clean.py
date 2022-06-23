from conan.api.conan_api import ConanAPIV2
from conans.cli.command import conan_command, OnceArgument
from conans.cli.output import Color, ConanOutput
from conans.client.userio import UserInput

info_color = Color.BRIGHT_WHITE
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

    out = ConanOutput()
    ui = UserInput(conan_api.config.get("core:non_interactive"))
    remote = conan_api.remotes.get(args.remote) if args.remote else None
    output_remote = remote or "Local cache"
    # Getting all the recipes
    recipes = conan_api.search.recipes("*/*", remote=remote)
    for recipe in recipes:
        out.writeln(f"{str(recipe)}", fg=recipe_color)
        # All RREVS and latest one
        all_rrevs = conan_api.list.recipe_revisions(recipe, remote=remote)
        latest_rrev = all_rrevs.pop(0) if all_rrevs else None
        if latest_rrev:
            # All PREVS and latest one (belonging to latest RREV)
            all_prevs = conan_api.search.package_revisions(f"{latest_rrev.repr_notime()}:*#*", remote=remote)
            latest_prev = all_prevs.pop(0) if all_prevs else None
            if latest_prev:
                out.writeln(f"  Keeping latest PREV: {latest_prev.repr_notime()} [{output_remote}]", fg=info_color)
            # Removing all the earlier package revisions of the latest recipe revision
            if all_prevs and confirmation(f"Remove all the {latest_rrev.repr_notime()} package revisions but the last one ?"):
                for prev in all_prevs:
                    conan_api.remove.package(prev, remote=remote)
                    out.writeln(f"  Removed PREV: {prev.repr_notime()} [{output_remote}]", fg=removed_color)
        # Removing all the earlier recipe revisions (and its packages)
        if all_rrevs and confirmation(f"Remove all the {str(recipe)} recipe revisions but the last one ?"):
            for rrev in all_rrevs:
                conan_api.remove.recipe(rrev, remote=remote)
                out.writeln(f"  Removed RREV: {rrev.repr_notime()} [{output_remote}]", fg=removed_color)
