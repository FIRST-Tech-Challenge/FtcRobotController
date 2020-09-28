# This the MechanicalManiacs Skystone repo

## !!!!!ALWAYS MAKE AN ISSUE BEFORE YOU DO SOMETHING!!!!!

## When making a branch please use the right format for naming
  git checkout -b "[type of branch]-[issue#]"

## Before Commiting please use linting

  For Windows: gradlew lint

  For Mac: ./gradlew lint
  
## When writing a commit message please use the right format

  git commit -m "feature-[issue#]: [Your commit message]"
  
## Please know the different types of branches
  
  Feature Branches: Feature branches are for big additions.
  
  Release Branches: Release branches are for last minute changes before a release that must be done to the develop branch.
  
  Hotfix Branches: Hotfix branches are for last minute changes to the working version of the code.

## Merge the feature, releases, and hotfixes with the right branches

  Feature and Release branches merge into develop
  
  Hotfix branches merge with develop and master
