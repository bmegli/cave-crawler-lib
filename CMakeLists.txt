cmake_minimum_required(VERSION 3.0)

project(
    cave-crawler
)

# altenatively SHARED instead of STATIC for a shared library
add_library(cave-crawler STATIC cave_crawler.c)

install(TARGETS cave-crawler DESTINATION lib)
install(FILES cave_crawler.h DESTINATION include)

add_executable(cc-read-all examples/cc_read_all.c)
target_link_libraries(cc-read-all cave-crawler)

