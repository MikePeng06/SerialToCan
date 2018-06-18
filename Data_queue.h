/*
 * Implement Circular Queue for Data buffer
 */
#include <iostream>
#define MAX 100
using namespace std;
/*
 * Class Circular Queue
 */
class Data_Queue
{
  private:
    char *cqueue_arr;
    int front, rear;

  public:
    Data_Queue()
    {
        cqueue_arr = new char[MAX];
        rear = front = -1;
    }
    /*
         * Insert into Circular Queue
         */
    void insert(char item)
    {
        if ((front == 0 && rear == MAX - 1) || (front == rear + 1))
        {
            cout << "Queue Overflow \n";
            return;
        }
        if (front == -1)
        {
            front = 0;
            rear = 0;
        }
        else
        {
            if (rear == MAX - 1)
                rear = 0;
            else
                rear = rear + 1;
        }
        cqueue_arr[rear] = item;
        printf("inserted ");
    }
    /*
         * Delete from Circular Queue
         */
    void del()
    {
        if (front == -1)
        {
            cout << "Queue Underflow\n";
            return;
        }
        cout << "Element deleted from queue is : " << cqueue_arr[front] << endl;
        if (front == rear)
        {
            front = -1;
            rear = -1;
        }
        else
        {
            if (front == MAX - 1)
                front = 0;
            else
                front = front + 1;
        }
    }

    char *getArray()
    {
        return cqueue_arr;
    }

    char getFrontIndex()
    {
        return front;
    }

    char getRearIndex()
    {
        return rear;
    }

    //return true and store the data to the buf if successfully get completed data
    bool getData(char *buffer, int &numOfData)
    {
        if ((cqueue_arr[front] == 'A') && (cqueue_arr[rear] == '\n'))
        {
            cout << "OK";
            int queue_index = front;
            int buffer_index = 0;

            do
            {

                if (queue_index == MAX)
                    queue_index = 0;
                buffer[buffer_index] = cqueue_arr[queue_index];

                buffer_index++;
                queue_index++;
            } while (queue_index != rear);
            buffer[buffer_index] = cqueue_arr[queue_index];

            //delete the data in queue after storing to the buf
            for(int i = 0; i < buffer_index; i ++){
                del();
            }

            numOfData = queue_index;

            return true;
        }

        return false;
    }

    /*
         * Display Circular Queue
         */
    void display()
    {
        int front_pos = front, rear_pos = rear;
        if (front == -1)
        {
            cout << "Queue is empty\n";
            return;
        }
        cout << "Queue elements :\n";
        if (front_pos <= rear_pos)
        {
            while (front_pos <= rear_pos)
            {
                cout << cqueue_arr[front_pos] << "  ";
                front_pos++;
            }
        }
        else
        {
            while (front_pos <= MAX - 1)
            {
                cout << cqueue_arr[front_pos] << "  ";
                front_pos++;
            }
            front_pos = 0;
            while (front_pos <= rear_pos)
            {
                cout << cqueue_arr[front_pos] << "  ";
                front_pos++;
            }
        }
        cout << endl;
    }
};