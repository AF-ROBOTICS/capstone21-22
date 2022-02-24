import json

filename = '../measurement_files/PathCache.json'


def add_to_cache(phrase, x, y):
    cache_dict = {}

    try:
        f = open(filename)
        cache_dict = json.load(f)
        f.close()
    except OSError:
        pass

    f = open(filename, 'w')
    cache_dict[phrase] = [x, y]
    json.dump(cache_dict, f, indent=4)

def check_cache(phrase):
    cache_dict = {}
    try:
        f = open(filename)
        cache_dict = json.load(f)
        f.close()
    except OSError:
        pass

    return cache_dict.get(phrase, None)

if __name__ == '__main__':
    print('help')
    add_to_cache('AJT',
                 [1.288888888888889, 1.8888888888888888, 1.4388888888888889, 1.738888888888889, 1.588888888888889,
                  1.588888888888889, 2.2888888888888888, 2.2888888888888888, 2.588888888888889, 2.588888888888889,
                  2.588888888888889, 2.8888888888888893, 2.2888888888888888, 3.2888888888888888, 3.5888888888888886,
                  3.5888888888888886, 3.5888888888888886, 3.8888888888888893],
                 [0.0, 0.0, 0.5, 0.5, 1.0, 0.5, 0.0, 0.3, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0, 0.0, 0.5, 1.0, 1.0])
