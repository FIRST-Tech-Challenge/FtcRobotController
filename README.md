[![Android CI](https://github.com/The-Knights-of-Ni/FreightFrenzy/actions/workflows/build.yml/badge.svg)](https://github.com/The-Knights-of-Ni/FreightFrenzy/actions/workflows/build.yml)
# Freight Frenzy 2022
## About

This repository contains FTC Team 5206's repository for the Freight Frenzy (2021-2022) competition season.

## Branches

**master** - stable code here

**auto** - auto code

**dev-pid** - pid development

**gh-pages** - Github Pages *does not have actual code*

**latest** - Latest Code from all branches

**teleop** - teleop development

**vision** - vision code

**\*-hotfix** - hotfix branch, quick fix

## Releaseses

### Release Cadence

**Sunday** - Come up with the new features to work on during the week and add the issues to the the current project.

**Monday-Wednesday** - Work on features.

**Thursday** - Bugfixes only on master, features need to be on branches

**Friday** - Bugfixes only on master, features need to be on branches. Come up with the new features to work on next week and add the issues to the next project.

**Saturday** - Test on robot and fix bugs.

### Release Versioning

the releases are called _yyyy.mm.dd.p_ when _yyyy_ is the year _mm_ is the month and _dd_ is the day and _p_ is the patch number (the default is 0).

# Docs

To view the latest docs install python 3.8, 3.9, or 3.10 and run the following commands
`pip install sphinx`
`cd docs`
`sphinx-build -b html source/ html`

the docs index page is in `/docs/html/index.html`
