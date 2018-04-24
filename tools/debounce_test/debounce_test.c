#include "debounce_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// For our test implementation, we only do one row of keys
// Really, we should just do one key, but this better matches
// production code
//
// All our data will be on key 0 in this row.

// Run the program with 'v' as its only commandline arg for verbose mode (where we output more data)
// Run this program with 'i' as its only commandline arg to output the input data on one line
// 'c' as the only argument will print the number of presses and nothing else. Exit code is presses-releases
debounce_t db[1];

uint8_t print_summary = 0;
uint8_t print_press_count = 0;
uint8_t verbose = 0;
uint8_t extra_verbose = 0;
uint8_t print_input = 0;
uint16_t scan_counter =0;
uint8_t pin_data[40960] = { };
uint8_t debounced_data[40960] = {};
char comments[40960][80] = {};

uint16_t press_duration[1024] = {0};
uint16_t release_duration[1024] = {0};

uint16_t press_latency_counts[1024] = {0};
uint16_t release_latency_counts[1024] = {0};


#define BUFFERSIZE 81920
void get_input () {
    char buffer[BUFFERSIZE];

    if (verbose) {
        printf("Enter some keyscan data. 0 for off. 1 for on. Lines starting with # are ignored. All other data reported as an error\n");
        printf("This program expects data to come from STDIN\n");
    }

    while( fgets(buffer, BUFFERSIZE, stdin) ) { /* break with ^D or ^Z */
        if (buffer[0] == '#' ) {
            strcpy( comments[scan_counter], buffer+1 );  // Yes, this copies past the possible end of the comment
            comments[scan_counter+1][79]= '\0';
            continue;
        }
        for(uint16_t i = 0; (i< sizeof(buffer) && buffer[i+1] != 0)  ; i++) {
            if(buffer[i] == '#') {
                break;
            } else if(buffer[i] == '0') {
                pin_data[scan_counter++] = 0;
            } else if (buffer[i] == '1') {

                pin_data[scan_counter++] = 1;
            } else {
                if (verbose  && buffer[i] != ' ' && buffer[i] != '\n') {
                    printf("Bad data ignored: %c\n", buffer[i]);
                }
            }
        }
    }

}



int main(int argc,char *argv[]) {
    if(argc ==2 && *argv[1] == 'v')  {
        verbose = 1;
    } else if(argc ==2 && *argv[1] == 'd')  { // debug - extra verbosity, show every sample transition
        verbose =1;
        extra_verbose=1;
    } else if(argc ==2 && *argv[1] == 'i')  {
        print_input = 1;
    } else if (argc ==2 && *argv[1] =='c') {
        print_press_count =1;
        print_summary=1;
    } else if (argc == 2 && *argv[1] == 's') {
        print_summary=1;
        print_press_count =1;
    }

    get_input();

    uint16_t last_input_change = 0;
    uint16_t last_state_change = 0;
    uint8_t presses =0;
    uint8_t releases =0;
    uint8_t debounced_changes =0;

    memset(db, 0, sizeof(*db) * COUNT_OUTPUT);


    if (verbose) {
        printf("Got %d samples\n",scan_counter);
    }

    for (uint16_t sample = 0; sample < scan_counter; sample++) {

        uint8_t data_printed = 0;
        debounced_changes= debounce(pin_data[sample], db);

        if (verbose) {
            if (comments[sample][0] != 0) {
                printf("# %s", comments[sample]);
            }
        }
        if (extra_verbose) {
            printf("Sample %-3d Input: %d ",sample,pin_data[sample]);
            printf("State: %d", (db[0].state ));
            data_printed++;
        }
        if (pin_data[sample] != pin_data[sample-1]) {
            if (verbose) {
                printf(" - input changed to %d after %d cycles", pin_data[sample], sample-last_input_change);
                data_printed++;
            };
            last_input_change = sample;
        }



        if (debounced_changes) {
            uint16_t latency = sample - last_input_change;
            uint16_t last_duration = sample - last_state_change;
            if (!(db[0].state )) {


                press_duration[presses] =last_duration;
                release_latency_counts[releases] = latency;
                releases++;
                if (verbose) {
                    printf(" - release %d ",releases);
                    data_printed++;
                }
            } else {

                release_duration[releases] = last_duration;
                presses++;
                press_latency_counts[presses] = latency;
                if (verbose) {
                    printf(" - press %d ", presses);
                    data_printed++;
                }
            }

            if (verbose) {
                printf(" - state changed to %d with a latency of %d cycles (old state lasted %d cycles)",
                       db[0].state, latency, last_duration);
                data_printed++;
            }

            last_state_change = sample;
        }

        debounced_data[sample] = ( db[0].state ) ;

        if (verbose && data_printed)
            printf("\n");
    }

    release_latency_counts[0]=0; // This is a hack

    if (verbose || print_input) {
        printf("Raw input data: (40 ms per line)\n");
        for(uint16_t i =0; i< scan_counter; i++) {
            printf("%d",pin_data[i]);
            if ((i+1)% 5 ==0)
                printf(" ");
            if ((i+1)%80==0)
                printf("\n");
        }
        printf("\n");
    }

    if (verbose ) {

        printf("Debounced data: (40ms per line)\n");
        for(uint16_t i =0; i< scan_counter; i++) {
            printf("%d",debounced_data[i]);
            if ((i+1)% 5 ==0)
                printf(" ");
            if ((i+1)%80==0)
                printf("\n");
        }
        printf("\n");
    }

    if (print_press_count) {
        printf("%d\n",presses);
    }
    if (verbose) {
        printf("# Total presses: %d Total releases: %d\n",presses,releases);
        printf("# Press/release timings:\n");
    }
    if (verbose || print_summary) {
        printf("#           Press:");
        for(uint8_t i = 0; (i<= presses||i<=releases); i++) {
            printf(" %4d",i);
        }
        printf("\n");





        printf("#   Press latency:");
        for(uint8_t i = 0; (i<=presses); i++) {
            printf(" %4d",press_latency_counts[i] );
        }
        printf("\n");
        printf("#        Held for:");
        for(uint8_t i = 0; (i<=presses); i++) {
            printf(" %4d",press_duration[i] );
        }

        printf("\n");






        printf("# Release latency:");
        for(uint8_t i = 0; (i<=releases); i++) {
            printf(" %4d",release_latency_counts[i] );
        }
        printf("\n");
        printf("#    Released for:");
        for(uint8_t i = 0; (i<=releases); i++) {
            printf(" %4d",release_duration[i] );
        }



        printf("\n");
    }

    if (print_press_count) {
        exit(presses-releases);
    }

    exit (0);
}
