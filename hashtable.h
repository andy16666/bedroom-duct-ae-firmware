/*
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */

// Hashtable with external chaining and arbitrary keys. 
// Author: Andrew Somerville 
#ifndef HASHTABLE_HH
#define HASHTABLE_HH
#define hashtable_t struct hashtable_t_t
#define hashtable_node_t struct hashtable_node_t_t
#include<sys/types.h>
#include<stdint.h>

// Constructor 
hashtable_t* create_hashtable(int store_size);

struct hashtable_t_t {
	int count;
	int store_size;
	hashtable_node_t **store;	

	// Methods 
	void  (*add)      (hashtable_t *h, void *key, size_t key_length, void *item);
	void* (*remove)   (hashtable_t *h, void *key, size_t key_length);
	void* (*get)      (hashtable_t *h, void *key, size_t key_length);
	int   (*is_empty) (hashtable_t *h);
	void  (*destroy)  (hashtable_t *h);	
};

struct hashtable_node_t_t {
	size_t key_length;  
	void *key;	
	void *item;
	hashtable_node_t *next;
};

// Private
static void  ___hashtable_add      (hashtable_t *h, void *key, size_t key_length, void *item);
static void* ___hashtable_remove   (hashtable_t *h, void *key, size_t key_length);
static void* ___hashtable_get      (hashtable_t *h, void *key, size_t key_length);
static int   ___hashtable_is_empty (hashtable_t *h);
static void  ___hashtable_destroy  (hashtable_t *h);
static int   ___hashtable_compare_keys(void *key, size_t key_len, void* key1, size_t key_len1);
static unsigned int ___hashtable_hash(void *key, size_t key_len, int max_key);

#endif
