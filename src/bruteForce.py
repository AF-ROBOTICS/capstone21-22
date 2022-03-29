# test basis for filling out JSON file with dest points for all combinations of letters
import PathBuild
import DynaLet


# iterate over uppercase ascii range [65,190] for 5 letters
def make_list():
    words = []
    j = k = l = m = -1
    for i in range(12356630):  # not the true number of iterations
        word = []
        if i % 26 == 0 and i != 0:
            j += 1
            if j % 26 == 0 and j != 0:
                k += 1
                if k % 26 == 0 and k != 0:
                    l += 1
                    if l % 26 == 0 and l != 0:
                        m += 1
        word += chr(i % 26 + 65)
        if j >= 0:
            word += chr(j % 26 + 65)
        if k >= 0:
            word += chr(k % 26 + 65)
        if l >= 0:
            word += chr(l % 26 + 65)
        if m >= 0:
            word += chr(m % 26 + 65)
        word.reverse()
        words.append(''.join(word))
        # if ''.join(word) == 'ZZZZZ':
        #     print(i)
        #     return
    return words


if __name__ == '__main__':
    words = make_list()
    print('made words')
    for word in words:
        x, y, wrd = DynaLet.custom_word(word)
        starts, ends = PathBuild.pack_to_points(x, y)
        x_pts, y_pts = PathBuild.build_path(starts, ends)
        if len(x_pts):  # only do this if it worked
            PathBuild.add_to_cache(word.upper(), x_pts, y_pts)
            print(f"added \'{word}\' to cache")
