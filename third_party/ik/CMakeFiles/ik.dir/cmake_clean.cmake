file(REMOVE_RECURSE
  "ik.a"
  "ik.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/ik.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
