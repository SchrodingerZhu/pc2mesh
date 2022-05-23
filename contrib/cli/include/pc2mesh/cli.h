#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    struct CliData {
        bool is_poisson;
        union {
            struct {
                char * input_file;
                char * output_file;
                size_t depth;
                double width;
                double scale;
                bool linear_fit;
                ssize_t num_threads;
                double filter;
            } poisson;

            struct {
                char * input_file;
                char * output_file;
                double * radii;
                size_t radii_size;
            } ball_pivoting;
        };
    };

    CliData pc2mesh_create_cli();
    void pc2mesh_clean_cli(CliData*);

#ifdef __cplusplus
}
#endif