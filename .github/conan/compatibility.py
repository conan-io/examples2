from conan.tools.build import supported_cppstd, supported_cstd


def cppstd_compat(conanfile):
    """Compute compatibility factors for different cppstd and cstd versions.
       By default, different compiler.cppstd and compiler.cstd are compatible.
       Returns a list of lists of dicts, each sublist is a potential factor with different settings.
    """
    extension_properties = getattr(conanfile, "extension_properties", {})
    compiler = conanfile.settings.get_safe("compiler")
    compiler_version = conanfile.settings.get_safe("compiler.version")
    cppstd = conanfile.settings.get_safe("compiler.cppstd")
    if not compiler or not compiler_version:
        return []
    factors = []  # List of list, each sublist is a potential combination
    if cppstd is not None and extension_properties.get("compatibility_cppstd") is not False:
        cppstd_possible_values = supported_cppstd(conanfile)
        if cppstd_possible_values is None:
            conanfile.output.warning(f'No cppstd compatibility defined for compiler "{compiler}"')
        else: # The current cppst must be included in case there is other factor
            factors.append([{"compiler.cppstd": v} for v in cppstd_possible_values])

    cstd = conanfile.settings.get_safe("compiler.cstd")
    if cstd is not None and extension_properties.get("compatibility_cstd") is not False:
        cstd_possible_values = supported_cstd(conanfile)
        if cstd_possible_values is None:
            conanfile.output.warning(f'No cstd compatibility defined for compiler "{compiler}"')
        else:
            factors.append([{"compiler.cstd": v} for v in cstd_possible_values if v != cstd])
    return factors


def compatibility(conanfile):
    """Compute all compatibility settings combinations for a given conanfile.
       Returns a list of dicts with "settings" key containing a list of (key, value) tuples.
    """
    factors = cppstd_compat(conanfile)

    # MSVC 194->193 fallback compatibility
    compiler = conanfile.settings.get_safe("compiler")
    compiler_version = conanfile.settings.get_safe("compiler.version")
    if compiler == "msvc":
        msvc_fallback = {"194": "193"}.get(compiler_version)
        if msvc_fallback:
            factors.append([{"compiler.version": msvc_fallback}])

    # Apple Clang 17->13 fallback compatibility
    if compiler == "apple-clang" and compiler_version == "17":
        factors.append([{"compiler.version": "13"}])

    # Combine factors to compute all possible configurations
    combinations = _factors_combinations(factors)
    # Final compatibility settings combinations to check
    return [{"settings": [(k, v) for k, v in comb.items()]} for comb in combinations]


def _factors_combinations(factors):
    """Given a list of factors (each factor is a list of dicts), compute all combinations.
       Returns a list of dicts with all combinations of the input factors.
    """
    combinations = []
    for factor in factors:
        if not combinations:
            combinations = factor
            continue
        new_combinations = []
        for comb in combinations:
            for f in factor:
                new_comb = comb.copy()
                new_comb.update(f)
                new_combinations.append(new_comb)
        combinations.extend(new_combinations)
    return combinations