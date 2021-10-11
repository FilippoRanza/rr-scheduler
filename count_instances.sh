#! /bin/bash


sqlite3 test-db_3.db 'SELECT COUNT(*) FROM instances WHERE status = 1'
for i in {2..6} ; do
    sqlite3 test-db_3.db "SELECT COUNT(*) FROM instances WHERE status = 1 and arm_count = $i"
done
