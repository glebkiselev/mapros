(define (problem BLOCKS-1-1) (:domain blocks)
(:objects
	block-a - block
	block-c - block
	block-b - block
	block-d - block
    ag1 - agent
    ag2 - agent
    big - size
    small - size
)
(:init
	(handempty ag1)
	(handempty ag2)
	(clear block-a)
	(clear block-b)
	(clear block-c)
	(clear block-d)
	(onground block-a)
	(onground block-b)
	(onground block-c)
	(onground block-d)
	(blocktype small block-a)
	(blocktype small block-b)
	(blocktype big block-c)
	(blocktype big block-d)
)
(:goal
	(and
	    (handempty ag1)
	    (handempty ag2)
		(on block-a block-b)
		(on block-c block-d)
        (blocktype small block-a)
        (blocktype small block-b)
        (blocktype big block-c)
        (blocktype big block-d)
        (clear block-a)
        (clear block-c)
        (onground block-b)
        (onground block-d)

	)
)

(:constraints
    (and

        (and (always (forall (?x - block)
            (implies (blocktype small ?x)(holding ag1 ?x))))
        )
        (and (always (forall (?x - block)
            (implies (blocktype big ?x)(holding ag2 ?x))))
        )

    )
)
)

