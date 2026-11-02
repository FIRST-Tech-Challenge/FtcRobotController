# Naming Conventions
--- -



### Tags
> Follow this naming convention: **v[Major].[Minor]-[Event]**

| Tag Structure          | Example              | Used For                                                                     |
|------------------------|----------------------|------------------------------------------------------------------------------|
| Scrimmage / Meet Build | `v1.0-meet1`         | A stable code package ready for league meets or scrimmage days.              |
| Qualifier Build        | `v2.0-qualifier`     | Code used at formal regional qualifying tournaments.                         |
| Interstate / Regionals | `v3.0-regionals`     | High-stakes tournament builds with locked-in autonomous coordinates.         |
| Emergency Patch        | `v2.1-qualifier-fix` | A fast fix deployed mid-tournament when a servo map changes between matches. |


### Branches
> Follow this naming convention: **[type]/[what was added]**

| Branch Structure | Example               | Used For       |
|------------------|-----------------------|----------------|
| feature          | `feature/new-feature` | Development    |
| bugfix           | `bugfix/issue-123`    | Bug Fixes      |
| hotfix           | `hotfix/critical-bug` | Critical Fixes |
| release          | `release/v2.1.0`      | Releases       |
| docs             | `docs/README-update`  | Documentation  |



