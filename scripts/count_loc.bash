find \
 './Blacksmith/src' \
 './blacksmith-docs/docs' \
 './blacksmith-docs/src' \
 './scripts' \
 \( -not -name '*.java' \) -print0 | xargs --null wc -l
