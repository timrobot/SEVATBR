from prquadtree import *

class ParticleFilter:
    def __init__(self, box):
      self.pr_tree = PRQuadTree(box)
      self.image_box = box
      #create uniform sampling
      subdivisions = 25 #number of subdivisions of grid (one side)
      cell_size = box.half_size * 2.0 / subdivisions
      for x in range(subdivisions):
          for y in range(subdivisions):
              self.pr_tree.insert(Particle(x * cell_size, y * cell_size))

    def iterate(self, blobs):
        '''
        For each blob, it updates the points in the tree increasing the score of those
        which are within the bounding square of the blob
        '''
        for blob in blobs:
            #query with the bounding box
            half_size = max(blob.minRectWidth(), blob.minRectHeight())/2
            box = Box(Point(blob.minRectX(), blob.minRectY()), half_size)
            points = self.pr_tree.query_range(box)
            for p in points:
                p.score += 1

    def score(self, blob):
        '''
        Returns the sum of the scores of the points found within this blob
        '''
        half_size = max(blob.minRectWidth(), blob.minRectHeight())/2
        box = Box(Point(blob.minRectX(), blob.minRectY()), half_size)
        points = self.pr_tree.query_range(box)
        point_sum = 0
        for p in points:
            point_sum += p.score
        return point_sum

    def clear_scores(self):
        '''
        Resets all scores of blobs
        This should be used when changing the webcam view
        '''
        points = self.pr_tree.query_range(self.image_box)
        for p in points:
            p.score = 0
