# Contributing guide

<a name="commit-message-format"></a>
## Commit message format
The project uses [this](https://www.conventionalcommits.org/en/v1.0.0/)
commit message standard:
```gitcommit
<type>(<scope>): <subject>
<BLANK LINE>
[optional body]
<BLANK LINE>
[optional footer]
```


<a name="git-tag-format"></a>
## Git Tag format
A tag is set on the commit for the next version release in [Semantic
Versioning](https://semver.org/) format: `MAJOR.MINOR.PATCH` with the prefix
`v` (e.g., `v0.1.2`).
```sh
git tag -a 'v0.1.2' -m "Fix v0.1.2"
git push --tags
```

When releasing a new version, all changes must be documented in
[CHANGELOG.md](CHANGELOG.md)


## Обновление конфига
```sh
. ~/playground/esp-idf/export.sh
idf.py save-defconfig
git add sdkconfig.defaults
```
