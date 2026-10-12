# Releasing network_bridge

The manual **Bloom release** workflow uses
[at-wat/bloom-release-action](https://github.com/at-wat/bloom-release-action)
unmodified. It pushes packaging to
[network_bridge-release](https://github.com/ros2-gbp/network_bridge-release)
and opens PRs against `ros/rosdistro`. The ROS buildfarm builds the Debian packages
after those PRs merge. This workflow publishes releases; it has no dry-run mode.

## One-time setup

1. Confirm your GitHub account (`brow1633`) has write access to the release repository.
2. Create a classic personal access token with `public_repo` and `workflow` scopes,
   as required by the action. Choose an expiration you can maintain, and authorize
   it for the release organization if GitHub requests that.
3. Store it as **BLOOM_RELEASE_TOKEN** under this repository's
   **Settings → Secrets and variables → Actions → New repository secret**.
   The built-in `GITHUB_TOKEN` cannot write to the separate release repository.
4. In the release repository's `master` branch, configure each intended track in
   `tracks.yaml` for unattended release:

   ```yaml
   version: :{auto}
   devel_branch: main
   release_tag: :{version}
   ```

   Existing tracks currently use `version: :{ask}`. Change that to `:{auto}`;
   keep the other track settings. Bloom will discover the version in upstream
   main's `package.xml` and release its matching numeric tag.

## Each release

1. Merge the changes and ensure CI is green.
2. Update `package.xml` and `CHANGELOG.rst` on main with the new version, for example
   `3.1.0`, and tag that commit **3.1.0**. Push the tag to
   `brow1633/network_bridge`. Tag creation is deliberately disabled in the action.
3. Check [ROS distribution status](https://github.com/ros/rosdistro/blob/master/index-v4.yaml).
   Select only `active` or `rolling` distributions with an existing release track.
   The default list is a convenience, not an automatically maintained support list.
4. Open **Actions → Bloom release → Run workflow**, choose **main**, and enter the
   space-separated distribution names. Keep main's package version at the tagged
   release version until Bloom finishes; the action discovers it from main.
5. Review the generated `ros/rosdistro` PRs and monitor the buildfarm after merge.

The action runs distributions sequentially because they share a release repository.
An initial release to a new distribution still needs manual Bloom track setup.
If a run partially succeeds, inspect its rosdistro PRs and logs before retrying;
Bloom may increment the packaging revision for distributions already processed.
