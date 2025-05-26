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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "hashtable.h"

hashtable_t* create_hashtable(int store_size) {
	hashtable_t *h = (hashtable_t *)malloc(sizeof(hashtable_t));
	
	h->store_size = store_size; 
	h->store = calloc(store_size, sizeof(hashtable_node_t*)); 
	h->count = 0;

	h->add      = ___hashtable_add; 
	h->get      = ___hashtable_get; 
	h->remove   = ___hashtable_remove; 
	h->is_empty = ___hashtable_is_empty; 
	h->destroy  = ___hashtable_destroy; 
	
	return h;
}

void ___hashtable_destroy(hashtable_t *h) {
	int i; 
	for (i = 0; i < h->store_size; i++) {
		hashtable_node_t* item; 
		while((item = h->store[i])) {
			h->store[i] = item->next; 
			free(item); 
			h->count--; 
		}
	} 
	free(h);
}

void  ___hashtable_add(hashtable_t *h, void *key, size_t key_length, void *item) {
	hashtable_node_t *node = (hashtable_node_t *)malloc(sizeof(hashtable_node_t));
		
	node->key_length = key_length; 
	node->key        = key; 
	node->item       = item;
	node->next       = NULL;
	
	int i = ___hashtable_hash(key, key_length, h->store_size); 
	
	hashtable_node_t **location = h->store + i; 

	while(*location) {	
		location = &((*location)->next);		
	}
	
	*location = node; 	

	h->count++;
}

void* ___hashtable_remove(hashtable_t *h, void *key, size_t key_length) {
	int i = ___hashtable_hash(key, key_length, h->store_size); 

	hashtable_node_t **node_location = h->store + i; 
	hashtable_node_t  *node          = *node_location; 
	while(node && !___hashtable_compare_keys(key, key_length, node->key, node->key_length)) {
		node_location = &(node->next); 
		node          = *node_location; 
	}

	if (node) {
		void *item = node->item; 
		*node_location = node->next; 
		h->count--; 
		free(node); 
		return item; 
	}
	else {
		return NULL; 
	}
	
}

void* ___hashtable_get(hashtable_t *h, void *key, size_t key_length) {
	int i = ___hashtable_hash(key, key_length, h->store_size); 

	hashtable_node_t *node = h->store[i]; 
	while(node && !___hashtable_compare_keys(key, key_length, node->key, node->key_length)) {
		node = node->next; 
	}

	if (node) return node->item; 
	else      return NULL; 
}

int ___hashtable_is_empty (hashtable_t *h) {
	return h->count == 0;
}

int ___hashtable_compare_keys(void *key, size_t key_len, void* key1, size_t key_len1) {
	if (key_len != key_len1) return 0; 
	return memcmp(key, key1, key_len) == 0; 
}

unsigned int ___hashtable_hash(void *key, size_t key_len, int max_key) {
	unsigned int hash = 1;
	int i;
	for(i = 0; i < key_len; i++) {		
		hash = (hash << 5) ^ ((char *)key)[i] ^ hash;
	} 

	return (hash + max_key) % max_key;
}
