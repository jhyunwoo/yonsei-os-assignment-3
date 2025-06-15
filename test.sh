make clean
make

./vmsim 10 5 FIFO < input_example.txt > ./test_result/output_example_fifo.txt
./vmsim 10 5 LRU < input_example.txt > ./test_result/output_example_lru.txt
./vmsim 10 5 LFU < input_example.txt > ./test_result/output_example_lfu.txt
./vmsim 10 5 S3FIFO < input_example.txt > ./test_result/output_example_s3fifo.txt

diff ./test_answer ./test_result --color