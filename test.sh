make clean
make
./vmsim 10 5 S3FIFO < input_example.txt > ./test_result/output_example_s3fifo.txt

diff ./test_answer ./test_result --color