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
        """
        Returns True if "--force" was passed or if user answers "yes" to the question message, otherwise, False
        """
        return args.force or ui.request_boolean(message)

    def remove_all_recipe_revisions_but_latest_one(rrevs):
        """
        Remove all the given recipes through ConanAPIV2.remove.recipe but the latest one (the first on the list given)
        """
        # Removing all the earlier recipe revisions (and its packages)
        if len(rrevs) > 1 and confirmation(f"Remove all the {str(recipe)} recipe revisions but the last one?"):
            for rrev in rrevs[1:]:
                conan_api.remove.recipe(rrev, remote=remote)
                out.writeln(f"  Removed RREV: {rrev.repr_notime()} [{output_remote}]", fg=removed_color)

        return rrevs[0] if rrevs else None

    def remove_all_package_revisions_but_latest_one(prevs):
        """
        Remove all the given packages through ConanAPIV2.remove.package but the latest one (the first on the list given)
        """
        # Removing all the earlier package revisions of the latest recipe revision
        if len(prevs) > 1 and confirmation(f"Remove all the {latest_rrev.repr_notime()} package revisions "
                                           f"but the last one?"):
            for prev in prevs[1:]:
                conan_api.remove.package(prev, remote=remote)
                out.writeln(f"  Removed PREV: {prev.repr_notime()} [{output_remote}]", fg=removed_color)

        return prevs[0] if prevs else None

    out = ConanOutput()
    ui = UserInput(conan_api.config.get("core:non_interactive"))
    remote = conan_api.remotes.get(args.remote) if args.remote else None
    output_remote = remote or "Local cache"
    # Getting all the recipes
    recipes = conan_api.search.recipes("*/*", remote=remote)
    for recipe in recipes:
        out.writeln(f"{str(recipe)}", fg=recipe_color)
        # Get all the RREVs (recipe revisions)
        all_rrevs = conan_api.list.recipe_revisions(recipe, remote=remote)
        # Save the latest RREV and remove the rest ones
        latest_rrev = remove_all_recipe_revisions_but_latest_one(all_rrevs)
        if latest_rrev:
            # Get all the PREVs (package revisions) belonging to latest RREV
            all_prevs = conan_api.search.package_revisions(f"{latest_rrev.repr_notime()}:*#*", remote=remote)
            # Save the latest PREV and remove the rest ones
            latest_prev = remove_all_package_revisions_but_latest_one(all_prevs)
            if latest_prev:
                out.writeln(f"  Keeping latest PREV: {latest_prev.repr_notime()} [{output_remote}]", fg=info_color)
